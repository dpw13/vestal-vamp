#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <stm32h7xx_hal_dfsdm.h>
#include "dfsdm_stm32.h"
#include "fft_dma.h"
#include "audio.h"

LOG_MODULE_REGISTER(filter, LOG_LEVEL_INF);

static const struct device *const dfsdm_dev = DEVICE_DT_GET(DT_NODELABEL(dfsdm1));

/* The ADC raw sample buffer */
audio_raw_t adc_buffer[FFT_SIZE*2] __attribute__((__section__("SRAM4")));

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

int filter_start(void) {
	return dfsdm_stm32_start(dfsdm_dev, &adc_buffer[0], FFT_SIZE*2*sizeof(audio_raw_t), dfsdm_cb);
}

int filter_stats(void) {
	LOG_INF("%s: %d samples: %"PRId16":%"PRId16, dfsdm_dev->name,
		sample_count,
		adc_buffer[0],
		adc_buffer[FFT_SIZE*2 - 1]);

	return 0;
}
