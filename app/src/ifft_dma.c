#include <zephyr/cache.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/math_extras.h>
#include <math.h>
#include "audio.h"
#include "dac.h"
#include "dma.h"
#include "freq_buffer.h"
#include "math_support.h"
#include "settings.h"

#include <arm_math.h>
#include <arm_const_structs.h>

#define ENABLE_ANALYSIS	0

LOG_MODULE_REGISTER(ifft_dma, LOG_LEVEL_INF);

/**
 * The frequency data for the IFFT. This is calculated from two frequency blocks
 * in the big frequency buffer. Use Q31 here because the RFFT algorithms shift
 * down to avoid saturation. For a 1024 FFT, the result is that we only get about
 * 3 useful bits remaining if we stick with Q15. Note that we need 2 additional
 * samples for the IFFT to unpack the DC and Nyquist frequencies.
 */
static q31_t ifft_fd_buffer[2 * FFT_SIZE + 2] Z_GENERIC_SECTION("DTCM") __aligned(32);
/**
 * The buffer of raw DAC samples out. These need to be windowed and accumulated into
 * the DAC buffer.
 */
static q31_t ifft_td_q31_buffer[AUDIO_OUT_SAMPLE_CNT] Z_GENERIC_SECTION("DTCM") __aligned(32);
static q15_t ifft_td_q15_buffer[AUDIO_OUT_SAMPLE_CNT] __aligned(32);

/* A static buffer for accumulating phase. These are stored as sQ-1.32 in the range [-0.5,0.5). */
static q15_t phase_buf[FFT_SIZE] __aligned(32);

uint32_t ifft_mag_buf[10];

q15_t injected_phase;

static inline void inject_tone(uint16_t bin, uint16_t magnitude)
{
	ifft_fd_buffer[2 * bin + 0] = (q31_t)magnitude * arm_cos_q15(injected_phase);
	ifft_fd_buffer[2 * bin + 1] = (q31_t)magnitude * arm_sin_q15(injected_phase);

	/* Model phase offset due to window overlap */
	injected_phase += bin << 12;
}

/**
 * Interpolate and phase-unwrap the frequency sample at fractional index
 * `idx`. Stores into `ifft_fd_buffer`.
 * @param idx: Fractional index into frequency buffers.
 * @param pitch_shift: an unsigned 8.8 fixed-point frequency multiplier.
 * @param phase_reset_threshold: an unsigned Q1.15 threshold for the *square* of the
 *      magnitude. Below this threshold, the phase of a frequency bin will be reset.
 *
 * TODO: validate that positive-going (1 s/s) and negative-moving (-1 s/s) interpolation are
 * equivalent.
 */
static void interp_freq(frac_idx_t idx, uint16_t pitch_shift)
{
	uint32_t k;
	uint32_t idx_i = idx_ipart(idx);
	uint32_t idx_j = (idx_i + 1) & WINDOW_IDX_MASK;
	uint16_t frac_b = idx_fpart(idx);
	uint16_t frac_a = (0x7fff) - frac_b;

	struct polar_freq_data *src_buf[2] = {
		get_lt_buf(idx_i),
		get_lt_buf(idx_j),
	};

	/* Clear destination buffer */
	memset(&ifft_fd_buffer[0], 0, sizeof(ifft_fd_buffer));
	if (0) {
		/* TODO: Settings and debug */
		inject_tone(128, 0x7fff);
	}

	/* Interpolate DC real content
	 * Note that these equations are somewhat misleading. We are multiplying two q15s
	 * to get a q31.
	 */
	ifft_fd_buffer[0] = mult_q15_to_q31(src_buf[0]->mag[0], frac_a) +
		mult_q15_to_q31(src_buf[1]->mag[0], frac_b);
	/* Nyquist real value is packed into index 0 of phase. Unpack to the expected index. */
	ifft_fd_buffer[2 * FFT_SIZE] =
		mult_q15_to_q31(src_buf[0]->phase[0], frac_a) + mult_q15_to_q31(src_buf[1]->phase[0], frac_b);

	/* Calculate phase delta by subtracting the precalculated instantaneous phases.
	 * Only do this for actual frequency bins and not bin 0.
	 * TODO: Compare this naive approach to vectorized math.
	 */

	q15_t *phase_a = &src_buf[0]->phase[1];
	q15_t *phase_b = &src_buf[1]->phase[1];
	q15_t *mag_a = &src_buf[0]->mag[1];
	q15_t *mag_b = &src_buf[1]->mag[1];

	for (k = 1; k < FFT_SIZE; k++) {
		/* Step 1: Calculate delta phase as (signed) Q2.13. Range (-pi, pi] */

		/* Phase is already in units of bins */
		q15_t phase = *phase_b++ - *phase_a++;

		/* Step 1b: Subtract expected phase from overlapping windows. The phase should
		 * be 1/INV_OVERLAP cycles. As a sanity check, we see that if INV_OVERLAP=1,
		 * this does nothing. If INV_OVERLAP=2, this rotates odd bins 180 degrees.
		 */
		/* bin_offset = INV_OVERLAP*wrap(delta_phase/M_TWOPI - k/INV_OVERLAP, +-1/2) */
		phase -= k * (Q15_ONE / INV_OVERLAP);

		/* Step 2: compute instantaneous frequency as k + INV_OVERLAP*inst_phase_delta */
		q31_t inst_freq = (k << 16) + ((uint32_t)phase << INV_OVERLAP_BITS);

		/* Step 3: pitch shift.
		 *
		 * Realistically with FFT_SIZE=1024, this puts a limit of 6 bits (64x) for
		 * the magnitude of the pitch shift. That's still more than reasonable as
		 * that would make anything but the lowest frequencies above Nyquist.
		 */
		inst_freq = (inst_freq * pitch_shift) >> 8;

		/* Compute destination bin with rounding now that we have the target freq */
#if 0
		uint16_t dst_k = (inst_freq + (1 << 15)) >> 16;
#else
		uint16_t dst_k = k;
#endif
		if (dst_k > FFT_SIZE) {
			/* Frequency is above Nyquist, discard */
			continue;
		}

		/* Step 4: Compute instantaneous phase = window delay * inst_freq. This value
		 * should be Q-1.16 in the range [-0.5, 0.5)
		 */
		phase = (inst_freq * (Q15_ONE / INV_OVERLAP)) >> 16;
		/* Accumulate instantaneous phase from previous window */
		phase += phase_buf[dst_k];

		/* Step 5: Compute complex phasor for destination bin. */
		/* 5a: Compute interpolated magnitude. Everything up to this point
		 * has only dealt with phase.
		 */
		q31_t mag = mult_q15_to_q31(*mag_a++, frac_a) + mult_q15_to_q31(*mag_b++, frac_b);

		/* 5b: compute phasor from magnitude and phase and add into destination bin. */
		/* The q15 fast cos takes as its input sQ0.15 in the range [0, 1) and will
		 * internally add Q15_ONE to negative values, so we can pass in our instantaneous
		 * phase as-is.
		 *
		 * The 1999 reference code directly assigns phase and increments magnitude.
		 * Here instead we accumulate the complex phasor, incorporating both magnitude
		 * and phase.
		 */
		ifft_fd_buffer[2 * dst_k + 0] += mult_q31(mag, arm_cos_q15(phase) << 16); /* Real */
		ifft_fd_buffer[2 * dst_k + 1] += mult_q31(mag, arm_sin_q15(phase) << 16); /* Imag */
	}
}

/**
 * Calculate the instantaneous phase in cycles for the phase accumulator.
 *
 * @param phase_reset_threshold: an unsigned Q1.15 threshold for the *square* of the
 *      magnitude. Below this threshold, the phase of a frequency bin will be reset.
 */
static void recalc_phase(q15_t phase_reset_threshold)
{
	/* Compute final phase for next window */
	q31_t *src = &ifft_fd_buffer[0];
	q15_t *dst = &phase_buf[0];
	for (int k = 1; k < FFT_SIZE; k++) {
		q31_t mag = arm_cmplx_mag_sq_q31_once_ni(src);

		if (mag < phase_reset_threshold) {
			*dst++ = 0;
		} else {
			q31_t phase;
			arm_atan2_q31(src[1], src[0], &phase);
			/* TODO: check radix point */
			/* TODO: use CORDIC and DMA */
			*dst++ = (q15_t)((M_TWO_OVER_PI_Q31 * phase) >> 15);
		}
		src += 2;
	}
}

static void bypass_interp(frac_idx_t idx) {
	struct polar_freq_data *src_buf = get_lt_buf(idx_ipart(idx));

	q15_t *pphase = &src_buf->phase[1];
	q15_t *pmag = &src_buf->mag[1];

	ifft_fd_buffer[0] = src_buf->mag[0] << 16;
	ifft_fd_buffer[1] = 0;
	/* Nyquist real value is packed into index 0 of phase. Unpack to the expected index. */
	ifft_fd_buffer[2 * FFT_SIZE] = src_buf->phase[0] << 16;
	ifft_fd_buffer[2 * FFT_SIZE + 1] = 0;

	memset(ifft_mag_buf, 0, sizeof(ifft_mag_buf));
	for (int k = 1; k < FFT_SIZE; k++) {
		int bin = 31 - u32_count_leading_zeros(k);
		q15_t mag = *pmag++;
		q15_t phase = *pphase++;

		ifft_mag_buf[bin] += (uint16_t)mag;
		ifft_fd_buffer[2 * k + 0] = mult_q15_to_q31(mag, arm_cos_q15(phase)); /* Real */
		ifft_fd_buffer[2 * k + 1] = mult_q15_to_q31(mag, arm_sin_q15(phase)); /* Imag */
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
static void accum_ifft_output(void)
{
	int i;
	uint8_t idx = active_win;

	LOG_DBG("-> %p", &dac_buffer[idx * OVERLAP_SAMPLES]);

	/* To make this easy, independently handle each 1/INV_OVERLAP
	 * of the buffer. */
	for (i = 0; i < INV_OVERLAP - 1; i++) {
		/* This should be a saturating addition */
		arm_add_q15(&ifft_td_q15_buffer[i * OVERLAP_SAMPLES],
			    &dac_buffer[idx * OVERLAP_SAMPLES], &dac_buffer[idx * OVERLAP_SAMPLES],
			    OVERLAP_SAMPLES);
		idx = (idx + 1) & (2 * INV_OVERLAP - 1);
	}

	/* Overwrite last sequence of samples. This could be DMA. */
	memcpy(&dac_buffer[idx * OVERLAP_SAMPLES], &ifft_td_q15_buffer[i * OVERLAP_SAMPLES],
	       sizeof(audio_raw_t) * OVERLAP_SAMPLES);

	/* Increment active window */
	active_win = (active_win + 1) & (2 * INV_OVERLAP - 1);
}

static void analysis_td(void) {
#if ENABLE_ANALYSIS
	q31_t accum = 0;
	q15_t max = 0x8000;
	q15_t min = 0x7fff;

	for (int i=0; i<FFT_SIZE; i++) {
		accum += ifft_td_q15_buffer[i];
		max = ifft_td_q15_buffer[i] > max ? ifft_td_q15_buffer[i] : max;
		min = ifft_td_q15_buffer[i] < min ? ifft_td_q15_buffer[i] : min;
	}

	LOG_INF("max %04x/mean %08x.%04x/min %04x", (uint16_t)max, accum >> 10, (uint16_t)((accum & ~(~0 << 10)) << 6), (uint16_t)min);
#endif
}

static void analysis_fd(void) {
#if ENABLE_ANALYSIS
	int j = 2;
	LOG_INF("%d %08x %08xi | %08x %08xi | %08x %08xi | %08x %08xi | %08x %08xi %d",
		j - 2, ifft_fd_buffer[2*j - 4], ifft_fd_buffer[2*j - 3],
		ifft_fd_buffer[2*j - 2], ifft_fd_buffer[2*j - 1],
		ifft_fd_buffer[2*j], ifft_fd_buffer[2*j + 1],
		ifft_fd_buffer[2*j + 2], ifft_fd_buffer[2*j + 3],
		ifft_fd_buffer[2*j+4], ifft_fd_buffer[2*j+5], j + 2);
#endif
}

/* FFT support structures */
static arm_rfft_instance_q31 Srev Z_GENERIC_SECTION(DTCM);
/* Frequency buffer index */
static frac_idx_t current_sample_idx;

/* Work handler to process display updates */
void ifft_work_handler(struct k_work *work)
{
	/* Settings */
	uint16_t pitch_shift = (uint16_t)get_uint_setting(SETTINGS_VOCODER_PITCH_SHIFT);
	q15_t phase_reset_threshold = (q15_t)get_int_setting(SETTINGS_VOCODER_PHASE_RESET_THRESH);
	frac_idx_t tune = (frac_idx_t)get_int_setting(SETTINGS_VOCODER_TIME_SCALE);

	current_sample_idx += tune;
#if 0
	// LOG_DBG("Interp start 0x%08x", current_sample_idx);
	interp_freq(current_sample_idx, pitch_shift);
	// LOG_DBG("Recalc phase");
	recalc_phase(phase_reset_threshold);
#else
	bypass_interp(current_sample_idx);
#endif
	// LOG_DBG("FFT start %p to %p", &ifft_fd_buffer[0], &ifft_td_q31_buffer[0]);
	analysis_fd();
	arm_rfft_q31(&Srev, &ifft_fd_buffer[0], &ifft_td_q31_buffer[0]);
	/* Clip, scale, and convert */
	arm_clip_q31(&ifft_td_q31_buffer[0], &ifft_td_q31_buffer[0], (-1 << 24), (1 << 24),
		     FFT_SIZE);
	arm_shift_q31(&ifft_td_q31_buffer[0], 11, &ifft_td_q31_buffer[0], FFT_SIZE);
	arm_q31_to_q15(&ifft_td_q31_buffer[0], &ifft_td_q15_buffer[0], FFT_SIZE);
	analysis_td();
	/* Apply window in-place to ADC samples */
	// LOG_DBG("Window start");
	arm_mult_q15(&fft_window_func[0], &ifft_td_q15_buffer[0], &ifft_td_q15_buffer[0], FFT_SIZE);
	/* Accumulate new window into waveform */
	LOG_DBG("Accum into window %d", active_win);
	accum_ifft_output();
	LOG_DBG("Accum done");
}

/* Define the work handler */
K_WORK_DEFINE(ifft_work, ifft_work_handler);

void schedule_ifft(void)
{
	int ret = k_work_submit(&ifft_work);
	if (ret < 0) {
		LOG_ERR("Error submitting work: %d", ret);
	}
}

void claim_dac_samples(void *buf, uint32_t len)
{
	/* TODO: check that buffers are available and we're at the expected
	 * sequence step.
	 */
	LOG_DBG("DAC tx %p, active win %d", buf, active_win);
}

int ifft_init(void)
{
	/* Initialize buffers to make startup easier to debug */
	memset(&dac_buffer[0], 0, sizeof(dac_buffer));

	/* Initialize FFT vectors */
	arm_status st = arm_rfft_init_1024_q31(&Srev, 1, 1);
	if (st != ARM_MATH_SUCCESS) {
		LOG_ERR("Could not initialize IFFT %d", st);
		return (int)st;
	}

	/* Initialize buffer accounting */
	wr_idx = 0;

	/* Initialize phase accumulator */
	memset(&phase_buf[0], 0, sizeof(phase_buf));

	return 0;
}