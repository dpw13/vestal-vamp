#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <stm32h7xx_hal_dfsdm.h>
#include "dfsdm_stm32.h"
#include "fmac_stm32.h"
#include "fft_dma.h"
#include "ifft_dma.h"
#include "audio.h"

LOG_MODULE_REGISTER(filter, LOG_LEVEL_DBG);

static const struct device *const dfsdm_dev = DEVICE_DT_GET(DT_NODELABEL(dfsdm1));
static const struct device *const fmac_dev = DEVICE_DT_GET(DT_NODELABEL(fmac));

/* The ADC raw sample buffer */
audio_raw_t adc_buffer[FFT_SIZE*2] __attribute__((__section__("SRAM4")));
audio_raw_t dac_buffer[FFT_SIZE*2] __attribute__((__section__("SRAM4")));

/* TODO: move to stats */
static uint32_t sample_count;

static int dfsdm_cb(const struct device *dev, void *buffer, size_t sc) {
	sample_count += FFT_SIZE;
	LOG_DBG("dfsdm_cb");

	/* By the time we execute, the previous buffer is already being overwritten. We
	 * can only rely on the fact that the *new* buffer will be stable until the next
	 * callback.
	 */
	if (sc == 0) {
		/* First half of buffer */
		submit_adc_samples(&adc_buffer[0], FFT_SIZE);
		return 0;
	}

	/* Second half of buffer */
	submit_adc_samples(&adc_buffer[FFT_SIZE], FFT_SIZE);
	return 0;
}

static int fmac_cb(const struct device *dev, void *buffer, size_t sc) {
	sample_count += FFT_SIZE;
	LOG_DBG("fmac_cb");

	if (sc == 0) {
		/* First half of dac_buffer */
		claim_dac_samples(&dac_buffer[0], FFT_SIZE);
	} else {
                /* Second half of dac_buffer*/
		claim_dac_samples(&dac_buffer[FFT_SIZE], FFT_SIZE);
        }

	return 0;
}

uint32_t filter_get_dac_dma_addr(void) {
	return fmac_stm32_get_output_reg(fmac_dev);
}

int filter_gen_fir(void) {
	/* TODO: actual coefficients */
	uint16_t buf[16] = {0};
	return fmac_stm32_configure_fir(fmac_dev, &buf[0], ARRAY_SIZE(buf));
}

int filter_start(void) {
	filter_gen_fir();
	fmac_stm32_start(fmac_dev, &dac_buffer[0], 2*FFT_SIZE, fmac_cb);
	return dfsdm_stm32_start(dfsdm_dev, &adc_buffer[0], FFT_SIZE*2*sizeof(audio_raw_t), dfsdm_cb);
}

int filter_stats(void) {
	LOG_INF("%s: %d samples: %"PRId16":%"PRId16, dfsdm_dev->name,
		sample_count,
		adc_buffer[0],
		adc_buffer[FFT_SIZE*2 - 1]);

	return 0;
}
