#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stm32_ll_dac.h>
#include "audio.h"
#include "dac.h"
#include "ifft_dma.h"

LOG_MODULE_REGISTER(audio_dac, LOG_LEVEL_WRN);

static const struct device *const dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

uint16_t dac_buffer[2*AUDIO_OUT_SAMPLE_CNT] __attribute__((__section__("SRAM4")));

static uint32_t sample_count;

uint8_t dac_convert_cb(const struct device *dev, uint16_t sampling_index) {
        LOG_DBG("dac_convert_cb");

	if (sampling_index == 0) {
		/* First half of dac_buffer */
		claim_dac_samples(&dac_buffer[0], AUDIO_OUT_SAMPLE_CNT);
	} else {
                /* Second half of dac_buffer*/
		claim_dac_samples(&dac_buffer[AUDIO_OUT_SAMPLE_CNT], AUDIO_OUT_SAMPLE_CNT);

        }

        sample_count += AUDIO_OUT_SAMPLE_CNT;
        return 1;
}

const static struct dac_channel_cfg dac_cfg = {
        .channel_id = 2, // Channel is 1-indexed
        .resolution = 12,
        .buffered = true,
        .continuous = true,
        .callback = &dac_convert_cb,
        .buffer_base = &dac_buffer[0],
        .buffer_size = 2 * AUDIO_OUT_SAMPLE_CNT * sizeof(uint16_t),
        .trig_src = LL_DAC_TRIG_EXT_TIM6_TRGO,
};

int dac_init(void) {
        int ret;

        sample_count = 0;

        ret = dac_channel_setup(dac_dev, &dac_cfg);
        if (ret < 0) {
                LOG_ERR("Error during dac_channel_setup: %d", ret);
                return ret;
        }
        /* The first value must be written to the DAC to enable updates */
        dac_write_value(dac_dev, 1, 0);

        return 0;
}

int dac_stats(void) {
        LOG_INF("%s: %d samples", dac_dev->name, sample_count);
        return 0;
}