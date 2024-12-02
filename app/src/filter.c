#include <zephyr/logging/log.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <stm32h7xx_hal_dfsdm.h>
#include "math_support.h"
#include "dfsdm_stm32.h"
#include "fmac_stm32.h"
#include "fft_dma.h"
#include "ifft_dma.h"
#include "audio.h"
#include "filter.h"

LOG_MODULE_REGISTER(filter, LOG_LEVEL_INF);

static const struct device *const dfsdm_dev = DEVICE_DT_GET(DT_NODELABEL(dfsdm1));
static const struct device *const fmac_dev = DEVICE_DT_GET(DT_NODELABEL(fmac));

/* The ADC raw sample buffer */
audio_raw_t adc_buffer[FFT_SIZE*2] __attribute__((__section__("SRAM4"))) __attribute__ ((aligned (32)));;
/* The "DAC buffer" holds the uninterpolated DAC samples. We zero-pad
 * these values before DMA so this buffer doesn't need to be in SRAM4.
 */
audio_raw_t dac_buffer[FFT_SIZE*2] __attribute__ ((aligned (32)));;

uint16_t upsample_buffer[4096] __attribute__((__section__("SRAM4"))) __attribute__ ((aligned (32)));;

/* Calculate the number of DAC samples copied into the upsample buffer for each
 * half-transfer
 */
#define UPSAMPLE_SRC_SAMPLES	(ARRAY_SIZE(upsample_buffer)/(2*AUDIO_OVERSAMPLE))
#define UPSAMPLE_HALF_BUF	(ARRAY_SIZE(upsample_buffer)/2)

/* TODO: move to stats */
static uint32_t dfsdm_sample_count;
static uint32_t fmac_sample_count;
static uint16_t dac_sample_idx;

static int dfsdm_cb(const struct device *dev, int status) {
	dfsdm_sample_count += FFT_SIZE;
	LOG_DBG("dfsdm_cb");

	/* By the time we execute, the previous buffer is already being overwritten. We
	 * can only rely on the fact that the *new* buffer will be stable until the next
	 * callback.
	 */
	if (status == DMA_STATUS_BLOCK) {
		/* First half of buffer */
		submit_adc_samples(&adc_buffer[0], FFT_SIZE);
		return 0;
	} else if (status == DMA_STATUS_COMPLETE) {
		/* Second half of buffer */
		submit_adc_samples(&adc_buffer[FFT_SIZE], FFT_SIZE);
		return 0;
	} else {
		LOG_ERR("DMA error %d", status);
	}

	return 0;
}

uint8_t ping_pong_idx;

static int fmac_cb(const struct device *dev, int status) {
	fmac_sample_count += UPSAMPLE_SRC_SAMPLES;

	/* Copy DAC samples into upsampling buffer */
	uint16_t *src = (uint16_t *)&dac_buffer[dac_sample_idx];
	uint16_t *dst;

	/* TODO: bookkeeping */
	claim_dac_samples(&dac_buffer[dac_sample_idx], UPSAMPLE_SRC_SAMPLES);
	dst = ping_pong_idx == 0 ? &upsample_buffer[0] : &upsample_buffer[UPSAMPLE_HALF_BUF];

	LOG_DBG("fmac_cb %d %p->%p", status, src, dst);

	/* The upsample buffer is already filled with zeros, so just copy
	 * non-zero samples.
	 */
	for (uint16_t i = 0; i < UPSAMPLE_SRC_SAMPLES; i++) {
		/* We need to convert from a signed, bipolar representation to an
		 * unsigned, unipolar representation. This is because the DAC interprets
		 * its samples as unsigned, and there appears to be no good way to convert
		 * the data between the FMAC and the DAC (and the DAC provides no easy
		 * offset configuration). */
		*dst = (0x8000 ^ *src++) >> 1;
		dst += AUDIO_OVERSAMPLE;
	}

	dac_sample_idx = (dac_sample_idx + UPSAMPLE_SRC_SAMPLES) % ARRAY_SIZE(dac_buffer);
	ping_pong_idx = !ping_pong_idx;

	return 0;
}

uint32_t filter_get_dac_dma_addr(void) {
	return fmac_stm32_get_output_reg(fmac_dev);
}

#define FMAC_SINC_CENTER	(AUDIO_OVERSAMPLE*FMAC_SINC_ORDER - 1)

static inline float hanning_window(float phase) {
	return 0.5f * (1.0f + cosf(phase));
}

static inline float hamming_window(float phase) {
	return 0.54f + (0.46f * cosf(phase));
}

static inline float blackman_window(float phase) {
	return 0.42f + (0.5f * cosf(phase)) + (0.08f * cosf(2.0f*phase));
}

static inline float triangle_window(float phase) {
	return phase;
}

int filter_gen_fir(void) {
	/* For a first-order sinc and 16x oversampling, we would need coefficients for +/- 15 samples for
	 * a total of 31 samples or (2*oversampling - 1). For a second-order filter, we would add 16 coeffs
	 * above and 16 coeffs below or (4*oversampling - 1) = (2*order*oversampling - 1). The center index
	 * is 2*order*oversampling and will always be 1.0.
	 */
	int16_t buf[AUDIO_OVERSAMPLE*2*FMAC_SINC_ORDER - 1];

	/* Maximum positive value is as close to 1.0 as we can get. We may need to scale the remaining
	 * coefficients so this is exactly 1.0.
	 */
	buf[FMAC_SINC_CENTER] = 0x7fff;
	for (int i = 1; i <= FMAC_SINC_CENTER; i++) {
		float phase = M_PI_F*i/AUDIO_OVERSAMPLE;
		float window_phase = M_PI_F*i/(AUDIO_OVERSAMPLE*FMAC_SINC_ORDER);
		float coeff = sinf(phase)/phase;
		float win = hamming_window(window_phase);
		/* TODO: scaling could be cheap to do here to keep samples away from DAC rails but because
		 * of how we're converting everything to unsigned (and losing the top bit of precision) we
		 * also end up with an offset error.
		 */
		int16_t c = arm_float_to_q15_once_round(win*coeff);
		buf[FMAC_SINC_CENTER + i] = c;
		buf[FMAC_SINC_CENTER - i] = c;
	}
	return fmac_stm32_configure_fir(fmac_dev, &buf[0], ARRAY_SIZE(buf));
}

int filter_start(void) {
	int ret;

	/* Initialize interpolation buffer */
	//for (int i=0; i<ARRAY_SIZE(upsample_buffer); i++) {
	//	upsample_buffer[i] = 0x8000;
	//}
	memset(&upsample_buffer[0], 0, sizeof(upsample_buffer));
	ping_pong_idx = 0;

	ret = filter_gen_fir();
	if (ret) {
		LOG_ERR("Failed to generate and configure FIR filter");
		return ret;
	}

	/* Note that the buffer length is in bytes, not words */
	ret = fmac_stm32_start(fmac_dev, &upsample_buffer[0], sizeof(upsample_buffer), fmac_cb);
	if (ret) {
		LOG_ERR("Failed to start FMAC");
		return ret;
	}

	/* Same here, buffer length in bytes */
	ret = dfsdm_stm32_start(dfsdm_dev, &adc_buffer[0], sizeof(adc_buffer), dfsdm_cb);
	if (ret) {
		LOG_ERR("Failed to start DFSDM");
		return ret;
	}

	return 0;
}

int filter_stats(void) {
	LOG_INF("%s: %d samples", dfsdm_dev->name,
		dfsdm_sample_count);
	LOG_INF("%s: %d samples", fmac_dev->name,
		fmac_sample_count);

	return 0;
}
