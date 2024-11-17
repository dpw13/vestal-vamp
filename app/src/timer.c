#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "timer.h"

LOG_MODULE_REGISTER(audio_timer);

#define TIMER_NODE      DT_NODELABEL(timers6)
const struct device *const counter_dev = DEVICE_DT_GET(DT_CHILD(TIMER_NODE, counter));

int timer_init(void) {
        int ret;

	if (!device_is_ready(counter_dev)) {
		LOG_ERR("device not ready.\n");
		return -EBUSY;
	}

        uint32_t freq = counter_get_frequency(counter_dev);
        uint32_t ticks = freq/AUDIO_TIMER_HZ;
        uint32_t actual = freq/ticks;

        struct counter_top_cfg counter_top_cfg = {
                .ticks = ticks,
                .flags = COUNTER_TOP_CFG_ENABLE_TRIG,
        };

        LOG_INF("Timer %s base frequency %d Hz. Using %d ticks for %d Hz",
                counter_dev->name, freq, ticks, actual);

        ret = counter_set_top_value(counter_dev, &counter_top_cfg);

        if (ret < 0) {
                LOG_ERR("Failed to set counter top value: %d", ret);
        }
        return ret;
}

int timer_start(void) {
        int ret = counter_start(counter_dev);

        if (ret < 0) {
                LOG_ERR("Failed to start counter: %d", ret);
        }
        return ret;
}

int timer_stats(void) {
        int ret;
        uint32_t ticks;
        ret = counter_get_value(counter_dev, &ticks);

        if (ret < 0) {
                LOG_ERR("Failed to get counter value: %d", ret);
        } else {
                LOG_DBG("%s %d ticks", counter_dev->name, ticks);
        }

        return ret;
}

int timer_stop(void) {
        int ret;
        ret = counter_stop(counter_dev);

        if (ret < 0) {
                LOG_ERR("Failed to start counter: %d", ret);
        }
        return ret;
}