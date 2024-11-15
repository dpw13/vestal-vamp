#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stm32_ll_dac.h>
#include "dac.h"

LOG_MODULE_REGISTER(audio_dac);

static const struct device *const dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

/* The size of the buffer in samples */
#define AUDIO_OUT_SAMPLE_CNT	1024
#define AUDIO_OUT_BUF_CNT	2
#define AUDIO_OUT_BUF_SIZE	(AUDIO_OUT_BUF_CNT * AUDIO_OUT_SAMPLE_CNT)
static uint16_t buffer[AUDIO_OUT_BUF_SIZE] __attribute__((__section__("SRAM4")));

static uint32_t sample_count;

uint8_t dac_convert_cb(const struct device *dev) {
        sample_count += AUDIO_OUT_SAMPLE_CNT;
        return 1;
}

const static struct dac_channel_cfg dac_cfg = {
        .channel_id = 1, // Channel is 1-indexed
        .resolution = 12,
        .buffered = true,
        .callback = &dac_convert_cb,
        .buffer_base = &buffer[0],
        .buffer_size = AUDIO_OUT_BUF_SIZE * sizeof(uint16_t),
        .buffer_count = AUDIO_OUT_BUF_CNT,
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

        return 0;
}
