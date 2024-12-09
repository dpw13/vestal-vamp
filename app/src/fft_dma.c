#include <zephyr/cache.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <arm_math.h>
#include <arm_const_structs.h>

#include "audio.h"
#include "adc.h"
#include "dma.h"
#include "freq_buffer.h"
#include "math_support.h"

#define ENABLE_ANALYSIS	0

LOG_MODULE_REGISTER(fft_dma, LOG_LEVEL_INF);

/* The buffer of real ADC samples. Align to cache line size. */
/* There appears to be a bug somewhere that invaliding the data cache on
 * an unaligned value causes corruption of memory after this buffer.
 */
static q15_t fft_td_q15_buffer[FFT_SIZE] __aligned(32);
static q31_t fft_td_q31_buffer[FFT_SIZE] Z_GENERIC_SECTION(DTCM) __aligned(32);

/* A buffer to hold the raw complex frequency phasors. Note that the
 * algorithm requires an extra bin for DC/Nyquist frequencies. */
static q31_t fft_fd_q31_samples[2 * FFT_SIZE + 2] Z_GENERIC_SECTION(DTCM) __aligned(32);
static q15_t fft_fd_q15_samples[2 * FFT_SIZE + 2] __aligned(32);

static void fft_dma_callback(const struct device *dev, void *user_data, uint32_t channel,
			     int status);

/* Memory-to-memory burst size */
#define DMA_M2M_BURST 16

/**
 * The memory-to-memory DMA device and configuration for FFT input
 */
static struct dma_def fft_dma_def = {
	DT_DMA_DEF_BY_NAME(DT_PATH(zephyr_user), audio_in_dma, DMA_M2M_BURST, fft_dma_callback),
	.dma_blk_cfg = {
		.block_size = FFT_SIZE * sizeof(audio_raw_t),

		/* This initial value is overwritten, but set to non-null to avoid a warning
		 * in `dma_config`. */
		.source_address = (uint32_t)&adc_buffer[0],
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.source_reload_en = 0,

		.dest_address = (uint32_t)&fft_td_q15_buffer[0],
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_reload_en = 0,

		/* Manually set the FIFO threshold to 1/4 because the
		 * dmamux DTS entry does not contain fifo threshold
		 */
		.fifo_mode_control = 0,
		.next_block = NULL,
	}};

/**
 * The current sequence index
 */
static uint8_t active_seq;

/**
 * DMA parameters from ADC memory buffer to FFT buffer
 */
struct inst_seq {
	uint32_t dma_src; /*!< Source address for DMA transfer */
	uint32_t dma_dst; /*!< Destination address for DMA transfer */
	uint32_t dma_len; /*!< DMA transfer size in bytes */

	/* Whether the FFT is ready to run once the DMA is complete */
	bool fft_ready;
	/* Whether the next DMA operation can be executed immediately */
	bool dma_ready;
};

/**
 * Helper macro for enumerating DMA/FFT sequence steps.
 *
 * @param src_ival Which quarter of the buffer to start DMA at
 * @param dst_ival Which half of a partial block (frac=2) to transfer
 * @param len_ival Length of block to transfer in units of OVERLAP_SAMPLES
 * @param _fft_ready If true, schedule an FFT on the data once transferred
 * @param _dma_ready If true, schedule the next DMA once the FFT has completed
 */
#define SEQ_INST(src_ival, dst_ival, len_ival, _fft_ready, _dma_ready)                             \
	{                                                                                          \
		.dma_src = (uint32_t) & adc_buffer[(src_ival) * OVERLAP_SAMPLES],                  \
		.dma_dst = (uint32_t) & fft_td_q15_buffer[(dst_ival) * OVERLAP_SAMPLES],           \
		.dma_len = sizeof(uint16_t) * (len_ival) * OVERLAP_SAMPLES,                        \
		.fft_ready = _fft_ready, .dma_ready = _dma_ready,                                  \
	}

/**
 * The schedule of DMA transfers and FFT computation to do for audio in. This
 * assumes the ADC DMA delivers one full FFT worth of raw samples each time
 * `submit_adc_samples` is called, and that we compute FFTs with 75% overlap so
 * that there are 2 FFTs computed each time `submit_adc_samples` is called.
 */
static const struct inst_seq seq[] = {
	/* The first INV_OVERLAP windows are contiguous */
	SEQ_INST(0, 0, 4, true, false),
	SEQ_INST(1, 0, 4, true, true),
	SEQ_INST(2, 0, 4, true, true),
	SEQ_INST(3, 0, 4, true, true),
	/* Starting with the second window in the second half, we
	 * have to handle wraparound */
	SEQ_INST(4, 0, 4, true, false),
	SEQ_INST(5, 0, 3, false, true),
	SEQ_INST(0, 3, 1, true, true),
	SEQ_INST(6, 0, 2, false, true),
	SEQ_INST(0, 2, 2, true, true),
	SEQ_INST(7, 0, 1, false, true),
	SEQ_INST(0, 1, 3, true, true),
};

static inline void incr_seq(void)
{
	if (++active_seq == ARRAY_SIZE(seq)) {
		active_seq = 0;
	}
}

static inline void xfer_next_buffer(void)
{
	uint32_t src = seq[active_seq].dma_src;
	uint32_t dst = seq[active_seq].dma_dst;
	uint32_t len = seq[active_seq].dma_len;
	int ret;

	ret = dma_reload(fft_dma_def.dma_dev, fft_dma_def.channel, src, dst, len);
	if (ret < 0) {
		LOG_ERR("Could not reload DMA: %d", ret);
	}
	LOG_DBG("DMA started: %d B from %p to %p", len, (void *)src, (void *)dst);
}

void submit_adc_samples(void *buf, uint32_t len)
{
	/* TODO: check that buffers are available and we're at the expected
	 * sequence step.
	 */
	LOG_DBG("ADC rx, seq %d", active_seq);
	xfer_next_buffer();
}

void schedule_ifft(void);

/**
 * If the current state indicates that we can immediately start DMA for
 * the next state, do so, then advance buffer window. Call this once all
 * work is completed for this window.
 */
static void window_done(void)
{
	bool start_dma = seq[active_seq].dma_ready;
	incr_seq();
	if (start_dma) {
		xfer_next_buffer();
	}
}

static uint8_t fft_lcl_idx;

/**
 * Convert the complex FFT phasors to polar magnitude and phase. The polar
 * form is easier to work with to calculate phase difference and interpolate
 * magnitudes.
 *
 * IGNORE BELOW: For now we're just storing the magnitude and phase, not
 * the magnitude and normalized instaneous frequency.
 *
 * What we store in long-term external PSRAM is the instantaneous magnitude
 * and the frequency offset for each bin. The frequency offset is calculated
 * as the phase difference between subsequent FFTs (taking into account the
 * phase delay from overlapping sample windows) normalized to +/- 0.5 in
 * Q0.15 format.
 *
 * Calculating the magnitude is straight-forward, and we call the vectorized
 * CMSIS-DSP functions to generate them. To determine the frequency offset,
 * we must compute the angle of each FFT bin using atan2. Storing the previous
 * FFT's angles allows us to efficiently compute the delta.
 *
 * The intermediate buffer fft_freq_lcl then should contain the magnitude and
 * instantaneous phase of the two most recent FFTs. We store these vectors
 * serially rather than interleaved to enable the use of vectorized CMSIS
 * functions.
 *
 * @param dst The destination buffer in PSRAM for the computed magnitude and
 * 	instantaneous frequency offset.
 */
void precalc_polar_diff(struct polar_freq_data *dst)
{
	/* Compute magnitude */
	arm_cmplx_mag_q15(&fft_fd_q15_samples[2], &dst->mag[1], FFT_SIZE - 1);

	/* Pack real DC and Nyquist values into mag[0] and phase[0], respectively */
	dst->mag[0] = fft_fd_q15_samples[0];
	dst->phase[0] = fft_fd_q15_samples[2 * FFT_SIZE];

	/* Instantaneous phase. Only calculate for bins > 0, as 0 contains
	 * the DC and Nyquist real values. */
	q15_t *cmplx_src = &fft_fd_q15_samples[2];
	q15_t *dst_ptr = &dst->phase[1];
	for (int i = 1; i < FFT_SIZE; i++) {
		if (arm_atan2_q15(cmplx_src[1], cmplx_src[0], dst_ptr) != ARM_MATH_SUCCESS) {
			/* Reset phase if the complex source is zero */
			*dst_ptr = 0;
		}
		/* Normalize to +/- 0.5 bins. If we change to CORDIC this may need
		 * to be updated, though hopefully the atan2 and sin/cos use the
		 * same units, unlike arm_atan2_q15 and arm_sin/cos_q15
		 */
		*dst_ptr = ((q31_t)(65536.0f / M_PI_F) * dst_ptr[0]) >> 15;
		cmplx_src += 2;
		dst_ptr++;
	}
}

/* FFT support structures */
static arm_rfft_instance_q31 S Z_GENERIC_SECTION(DTCM);

/* The windowing function. Ideally this would live in flash, not SRAM. */
q15_t fft_window_func[FFT_SIZE] __aligned(32) Z_GENERIC_SECTION(DTCM);

static void analysis_td(void) {
#if ENABLE_ANALYSIS
	q31_t accum = 0;
	q15_t max = 0x8000;
	q15_t min = 0x7fff;
	q15_t max_diff = 0;

	for (int i=0; i<FFT_SIZE; i++) {
		accum += fft_td_q15_buffer[i];
		max = fft_td_q15_buffer[i] > max ? fft_td_q15_buffer[i] : max;
		min = fft_td_q15_buffer[i] < min ? fft_td_q15_buffer[i] : min;

		if (i > 0) {
			q15_t d = fft_td_q15_buffer[i] - fft_td_q15_buffer[i-1];
			if (d < 0)
				d = -d;
			if (d > max_diff)
				max_diff = d;
		}
	}

	LOG_INF("max %04x/mean %08x.%04x/min %04x delta %04x", (uint16_t)max, accum >> 10, (uint16_t)((accum & ~(~0 << 10)) << 6), (uint16_t)min, max_diff);
#endif
}

static void analysis_fd(void) {
#if ENABLE_ANALYSIS
	q15_t max_mag = 0x8000;
	int max_idx = 0;
	struct polar_freq_data *buf = &lt_buffer[wr_idx];

	for (int i=0; i<FFT_SIZE/2; i++) {
		if (buf->mag[i] > max_mag) {
			max_idx = i;
			max_mag = buf->mag[i];
		}
	}

	int j = max_idx < 2 ? 2 : max_idx;
	LOG_INF("Peak at %03i %d %04hx | %04hx | %04hx | %04hx | %04hx %d", max_idx,
		j - 2, buf->mag[j-2], buf->mag[j-1],
		buf->mag[j],
		buf->mag[j+1], buf->mag[j+2], j + 2);
	LOG_INF("            %d %04hx | %04hx | %04hx | %04hx | %04hx %d",
		j - 2, buf->phase[j-2], buf->phase[j-1],
		buf->phase[j],
		buf->phase[j+1], buf->phase[j+2], j + 2);
#endif
}

/* Work handler to process display updates */
void fft_work_handler(struct k_work *work)
{
	// LOG_DBG("Window start");
	/* Invalidate cache; the waveform present in fft_td_q15_buffer is new. */
	sys_cache_data_invd_range(&fft_td_q15_buffer[0], sizeof(q15_t) * FFT_SIZE);
	analysis_td();
	/* Apply window in-place to ADC samples */
	arm_mult_q15(&fft_window_func[0], &fft_td_q15_buffer[0], &fft_td_q15_buffer[0], FFT_SIZE);
	/* TODO: This is super inefficient */
	arm_q15_to_q31(&fft_td_q15_buffer[0], &fft_td_q31_buffer[0], FFT_SIZE);
	/* input time-domain samples are getting corrupted. Invalidate the cache in case a cache
	 * flush before the next DMA completes is causing corruption. */
	sys_cache_data_invd_range(&fft_td_q15_buffer[0], sizeof(q15_t) * FFT_SIZE);
	LOG_DBG("FFT start");
	arm_rfft_q31(&S, &fft_td_q31_buffer[0], &fft_fd_q31_samples[0]);
	/* TODO: This is super inefficient */
	/* Clip, scale, and convert */
	// arm_clip_q31(&fft_fd_q31_samples[0], &fft_fd_q31_samples[0], (-1 << 28), (1 << 28),
	// 2*FFT_SIZE+2); arm_shift_q31(&fft_fd_q31_samples[0], 2, &fft_fd_q31_samples[0],
	// 2*FFT_SIZE+2);
	arm_q31_to_q15(&fft_fd_q31_samples[0], &fft_fd_q15_samples[0], 2 * FFT_SIZE + 2);
	/* I assume we can write directly to the destination buffer. This may need to
	 * be an additional DMA instead. */
	precalc_polar_diff(&lt_buffer[wr_idx]);
	analysis_fd();
	/* Increment the write index */
	wr_idx = (wr_idx + 1) & WINDOW_IDX_MASK;
	// LOG_DBG("FFT done");
	window_done();
	/* Schedule IFFT work */
	/* TODO: We probably want to indicate which buffer was just written
	 * so the IFFT side can determine "now".
	 */
	schedule_ifft();
}

/* Define the work handler */
K_WORK_DEFINE(fft_work, fft_work_handler);

static inline void schedule_fft(void)
{
	int ret = k_work_submit(&fft_work);
	if (ret < 0) {
		LOG_ERR("Error submitting work: %d", ret);
	}
}

static void fft_dma_callback(const struct device *dev, void *user_data, uint32_t channel,
			     int status)
{
	/* TODO: book-keeping of buffers, check for overrun */
	// LOG_DBG("DMA rx");
	if (seq[active_seq].fft_ready) {
		schedule_fft();
	} else {
		window_done();
	}
}

/* Copied from arm_hanning_f32 */
static void arm_hanning_q15(q15_t *pDst, uint32_t blockSize)
{
	float32_t k = 2.0f / ((float32_t)blockSize);
	float32_t w;

	for (uint32_t i = 0; i < blockSize; i++) {
		w = PI * i * k;
		w = 0.5f * (1.0f - cosf(w));
		pDst[i] = arm_float_to_q15_once(w);
	}
}

int fft_init(void)
{
	int ret;

	/* Configure DMA for uncached SRAM4 buffer to cached SRAM1 buffer */
	fft_dma_def.dma_cfg.head_block = &fft_dma_def.dma_blk_cfg;
	ret = dma_config(fft_dma_def.dma_dev, fft_dma_def.channel, &fft_dma_def.dma_cfg);
	if (ret != 0) {
		LOG_ERR("Could not configure DMA: %d", ret);
		return ret;
	}

	/* Initialize FFT vectors */
	arm_status st = arm_rfft_init_1024_q31(&S, 0, 1);
	if (st != ARM_MATH_SUCCESS) {
		LOG_ERR("Could not initialize FFT %d", st);
		return (int)st;
	}

	/* Initialize window */
	arm_hanning_q15(&fft_window_func[0], FFT_SIZE);

	/* Initialize buffer accounting */
	wr_idx = 0;
	fft_lcl_idx = 0;

	return 0;
}