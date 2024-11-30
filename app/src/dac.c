#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stm32_ll_dac.h>
#include "audio.h"
#include "filter.h"
#include "dac.h"
#include "ifft_dma.h"

LOG_MODULE_REGISTER(audio_dac, LOG_LEVEL_WRN);

static const struct device *const dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

static uint32_t sample_count;

static struct dac_channel_cfg dac_cfg = {
        .channel_id = 2, // Channel is 1-indexed
        .resolution = 12,
        .internal = true,
        .buffered = true, /* May not need buffer for internal-only connection? */
        .continuous = true,
        .callback = NULL,
        .buffer_base = NULL,
        .buffer_size = 0,
        .trig_src = LL_DAC_TRIG_EXT_TIM6_TRGO,
};

int dac_init(void) {
        int ret;

        sample_count = 0;
        dac_cfg.buffer_base = (void *)filter_get_dac_dma_addr();

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