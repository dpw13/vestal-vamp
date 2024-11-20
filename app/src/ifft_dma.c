#include <zephyr/cache.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "audio.h"
#include "dac.h"
#include "dma.h"
#include "freq_buffer.h"
#include "math_support.h"

#include <arm_math.h>
#include <arm_const_structs.h>

LOG_MODULE_REGISTER(ifft_dma, LOG_LEVEL_INF);

/**
 * The frequency data for the IFFT. This is calculated from two frequency blocks
 * in the big frequency buffer.
 */
static q15_t ifft_buffer[2*FFT_SIZE] __attribute__((__section__("SRAM1")));
/**
 * The buffer of raw DAC samples out. These need to be windowed and accumulated into
 * the DAC buffer.
 */
static q15_t ifft_raw_buffer[2*AUDIO_OUT_SAMPLE_CNT] __attribute__((__section__("SRAM1")));

/* A static buffer for accumulating phase. These are stored as sQ-1.16 in the range [-0.5,0.5). */
static q15_t phase_buf[FFT_SIZE];

/* 1/(2*M_PI_F) in uQ-2.18 format */
static uint32_t inv_two_pi;

/**
 * Interpolate and phase-unwrap the frequency sample at fractional index
 * `idx`. Stores into `ifft_buffer`.
 * @param idx: Fractional index into frequency buffers.
 * @param pitch_shift: an unsigned 8.8 fixed-point frequency multiplier.
 * @param phase_reset_threshold: an unsigned Q1.15 threshold for the *square* of the 
 *      magnitude. Below this threshold, the phase of a frequency bin will be reset.
 * 
 * TODO: validate that positive-going (1 s/s) and negative-moving (-1 s/s) interpolation are
 * equivalent.
 */
static void interp_freq(frac_idx_t idx, uint16_t pitch_shift, q15_t phase_reset_threshold) {
        uint32_t k;
        uint32_t idx_i = idx_ipart(idx);
        uint32_t idx_j = (idx_i + 1) & WINDOW_IDX_MASK;
        uint32_t frac_a = idx_fpart(idx);
        uint32_t frac_b = (1u << 31) - frac_a;

        q15_t *src_buf[2] = {
                get_freq_buf(idx_i), get_freq_buf(idx_j),
        };

        /* Clear destination buffer */
        memset(&ifft_buffer[0], 0, sizeof(2*FFT_SIZE));

        /* Calculate phase delta. We calculate the phases of both buffers
         * and subtract them.
         * TODO: Compare speed of two arm_atan2_q15 vs complex vector 
         * division and a single arm_atan2_q15.
         * TODO: Compare this naive approach to vectorized math.
         */
        /* TODO: pre-calculate phase delta (freq offset) on input so that we
         * only need to calculate the phase of each bin once. Below we are calculating
         * the phase of each bin for *both* source buffers every window. If we store
         * magnitude and phase, interpolation here becomes relatively simple.
         */

        q15_t *src_a = src_buf[0];
        q15_t *src_b = src_buf[1];
        q15_t phase_a, phase_b;
        for (k=0; k < FFT_SIZE; k++) {
                /* Step 1: Calculate delta phase as (signed) Q2.13. Range (-pi, pi] */
                arm_atan2_q15(src_a[1], src_a[0], &phase_a);
                arm_atan2_q15(src_b[1], src_b[0], &phase_b);

                /* Step 1a: Divide by 2*pi to get normalized phase in units of bins per window.
                 * The range should be (-0.5, 0.5] cycles (0.16 fxp), representing a phase change
                 * *per window* of (-pi, pi]. This allows us to subtract the expected phase
                 * difference without having to explicitly wrap.
                 * 
                 * (phase_b - phase_a): sQ3.13
                 * *inv_two_pi: sQ0.31 (inv_two_pi should be uQ.18)
                 * output: sQ0.15
                 * 
                 * TODO: The CMSIS DSP functions appear to never round. Do we want to round here?
                 */
                phase_a = (q15_t)(((int32_t)phase_b - phase_a)*inv_two_pi >> 16);

                /* Step 1b: Subtract expected phase from overlapping windows. The phase should
                 * be 1/INV_OVERLAP cycles. As a sanity check, we see that if INV_OVERLAP=1,
                 * this does nothing. If INV_OVERLAP=2, this rotates odd bins 180 degrees.
                 */
                /* bin_offset = INV_OVERLAP*wrap(delta_phase/M_TWOPI - k/INV_OVERLAP, +-1/2) */
                phase_a -= k * (Q15_ONE / INV_OVERLAP);

                /* Step 2: compute instantaneous frequency as k + INV_OVERLAP*inst_phase_delta */
                q31_t inst_freq = (k << 16) + ((uint32_t)phase_a << INV_OVERLAP_BITS);

                /* Step 3: pitch shift.
                 *
                 * Realistically with FFT_SIZE=1024, this puts a limit of 6 bits (64x) for
                 * the magnitude of the pitch shift. That's still more than reasonable as
                 * that would make anything but the lowest frequencies above Nyquist.
                 */
                inst_freq = (inst_freq * pitch_shift) >> 8;

                /* Compute destination bin with rounding now that we have the target freq */
                uint16_t dst_k = (inst_freq + (1 << 15)) >> 16;
                if (dst_k > FFT_SIZE) {
                        /* Frequency is above Nyquist, discard */
                        continue;
                }

                /* Step 4: Compute instantaneous phase = window delay * inst_freq. This value
                 * should be Q-1.16 in the range [-0.5, 0.5)
                 */
                phase_a = (inst_freq * (Q15_ONE / INV_OVERLAP)) >> 16;
                /* Accumulate instantaneous phase from previous window */
                phase_a += phase_buf[dst_k];

                /* Step 5: Compute complex phasor for destination bin. */
                /* 5a: Compute interpolated magnitude. Everything up to this point
                 * has only dealt with phase.
                 *
                 * Note: These increment src_a and src_b!
                 */
                q15_t mag_a = arm_cmplx_mag_q15_once_ai(&src_a);
                q15_t mag_b = arm_cmplx_mag_q15_once_ai(&src_b);
                /* Interpolate into mag_a */
                mag_a = mag_a*frac_a + mag_b*frac_b;
                
                /* 5b: compute phasor from magnitude and phase and add into destination bin. */
                /* The q15 fast cos takes as its input sQ0.15 in the range [0, 1) and will
                 * internally add Q15_ONE to negative values, so we can pass in our instantaneous
                 * phase as-is.
                 * 
                 * The 1999 reference code directly assigns phase and increments magnitude.
                 * Here instead we accumulate the complex phasor, incorporating both magnitude
                 * and phase.
                 */
                ifft_buffer[2*dst_k+0] += mag_a*arm_cos_q15(phase_a); /* Real */
                ifft_buffer[2*dst_k+1] += mag_a*arm_sin_q15(phase_a); /* Imag */
        }

        /* Compute final phase for next window */
        src_a = &ifft_buffer[0];
        src_b = &phase_buf[0];
        for (k=0; k < FFT_SIZE; k++) {
                q15_t mag = arm_cmplx_mag_sq_q15_once_ni(src_a);

                if (mag < phase_reset_threshold) {
                        *src_b++ = 0;
                } else {
                        arm_atan2_q15(src_a[1], src_a[0], src_b++);
                }
                src_a += 2;
        }
}

/**
 * The current output window in the DAC sample buffer, from 0 to 2*INV_OVERLAP-1.
 */
static uint8_t active_win;

/** 
 * Accumulates the windowed output samples of the IFFT into the DAC
 * buffer. This involves accumulating all but the last OVERLAP_SAMPLES
 * into the buffer and overwriting the last OVERLAP_SAMPLES.
 */
static void accum_ifft_output(void) {
        int i;
        uint8_t idx = active_win;

        /* To make this easy, independently handle each 1/INV_OVERLAP
         * of the buffer. */
        for (i=0; i < INV_OVERLAP-1; i++) {
                arm_add_q15(&ifft_raw_buffer[i*OVERLAP_SAMPLES],
                        &dac_buffer[idx*OVERLAP_SAMPLES],
                        &dac_buffer[idx*OVERLAP_SAMPLES],
                        OVERLAP_SAMPLES);
                idx = (idx + 1) & (2*INV_OVERLAP - 1);
        }
        /* Overwrite last sequence of samples. This could be DMA. */
        memcpy(&ifft_raw_buffer[i*OVERLAP_SAMPLES],
                &dac_buffer[idx*OVERLAP_SAMPLES],
                sizeof(audio_raw_t)*OVERLAP_SAMPLES);

        /* Increment active window */
        active_win = (active_win + 1) & (2*INV_OVERLAP - 1);
}

/* FFT support structures */
static arm_rfft_instance_q15 Srev;
/* Frequency buffer index */
static frac_idx_t current_sample_idx;

/* Work handler to process display updates */
void ifft_work_handler(struct k_work *work) {
        /* Settings */
        uint16_t pitch_shift = (1u << 8); // no shift
        q15_t phase_reset_threshold = (1 << 2);
        frac_idx_t tune = FRAC_IDX_UNITY; // one in buffer per out buffer

        current_sample_idx += tune;
	LOG_DBG("Interp start 0x%08x", current_sample_idx);
        interp_freq(current_sample_idx, pitch_shift, phase_reset_threshold);
	LOG_DBG("FFT start %p to %p", &ifft_buffer[0], &ifft_raw_buffer[0]);
        arm_rfft_q15(&Srev, &ifft_buffer[0], &ifft_raw_buffer[0]);
	LOG_DBG("Window start");
	/* Apply window in-place to ADC samples */
	arm_mult_q15(&fft_window_func[0], &ifft_raw_buffer[0], &ifft_raw_buffer[0], FFT_SIZE);
        /* Accumulate new window into waveform */
	LOG_DBG("Accum into window %d", active_win);
	accum_ifft_output();
	LOG_DBG("Accum into window %d done", active_win);
}

/* Define the work handler */
K_WORK_DEFINE(ifft_work, ifft_work_handler);

void schedule_ifft(void) {
        int ret = k_work_submit(&ifft_work);
        if (ret < 0) {
            LOG_ERR("Error submitting work: %d", ret);
        }
}

void claim_dac_samples(void *buf, uint32_t len) {
        /* TODO: check that buffers are available and we're at the expected
         * sequence step.
         */
	LOG_DBG("DAC tx %p, active win %d", buf, active_win);
}

int ifft_init(void) {
        /* Initialize FFT vectors */
        arm_status st = arm_rfft_init_1024_q15(&Srev, 1, 1);
        if (st != ARM_MATH_SUCCESS) {
                LOG_ERR("Could not initialize IFFT %d", st);
                return (int)st;
        }

	/* Initialize buffer accounting */
	wr_idx = 0;

        /* Initialize constants */
        /* TODO: verify radix */
        inv_two_pi = (uint32_t)arm_float_to_q31_once(0.5f / M_PI_F);

        /* Initialize phase accumulator */
        for (int i=0; i<FFT_SIZE; i++) {
                phase_buf[i] = 0;
        }

        return 0;
}