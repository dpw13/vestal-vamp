#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "adc.h"

LOG_MODULE_REGISTER(audio_adc);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/* The size of the buffer in samples */
#define AUDIO_IN_SAMPLE_CNT	1024
static uint16_t buffer[AUDIO_IN_SAMPLE_CNT] __attribute__((__section__("SRAM4")));

static uint32_t sample_count;

enum adc_action adc_sequence_cb(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index) {
	sample_count += AUDIO_IN_SAMPLE_CNT/2;

	/* By the time we execute, the previous buffer is already being overwritten. We
	 * can only rely on the fact that the *new* buffer will be stable until the next
	 * callback.
	 */
	//LOG_INF("- %s %d: %"PRId16":%"PRId16" x %"PRId16":%"PRId16,
	//	dev->name, sampling_index,
	//	buffer[8], buffer[AUDIO_IN_SAMPLE_CNT/2 - 8],
	//	buffer[AUDIO_IN_SAMPLE_CNT/2 + 8], buffer[AUDIO_IN_SAMPLE_CNT - 8]);
	//LOG_INF("adc_sequence_cb %d: %d", sampling_index, sample_count);

	if (sampling_index > 0) {
		return ADC_ACTION_REPEAT;
	}

	return ADC_ACTION_CONTINUE;
}

const struct adc_sequence_options opts = {
	.callback = &adc_sequence_cb,
	.block_size = AUDIO_IN_SAMPLE_CNT,
	.continuous = true,
};

struct adc_sequence sequence = {
	.options = &opts,
	.buffer = &buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buffer),
};

int adc_init(void) {
	int ret;

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		ret = adc_is_ready_dt(&adc_channels[i]);
		if (!ret) {
			LOG_ERR("ADC controller device %s not ready", adc_channels[i].dev->name);
			return ret;
		}

		LOG_INF("ADC %s:%d: %d bits, vref %d, %d ticks, trig %d",
			adc_channels[i].dev->name, adc_channels[i].channel_id,
			adc_channels[i].resolution, adc_channels[i].vref_mv,
			adc_channels[i].channel_cfg.acquisition_time, adc_channels[i].channel_cfg.trig_src);
		ret = adc_channel_setup_dt(&adc_channels[i]);
		if (ret < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, ret);
			return ret;
		}
	}

	return 0;
}

int adc_start(void) {
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		err = adc_read_dt(&adc_channels[i], &sequence);
		if (err < 0) {
			LOG_ERR("Could not read (%d)", err);
			return err;
		}
		/* This is hacked together such that the read call returns
		 * before data is available, as we're in streaming mode.
		 */
	}

	return 0;
}

int adc_stats(void) {
	LOG_INF("%s: %d samples: %"PRId16":%"PRId16, adc_channels[0].dev->name,
		sample_count,
		buffer[0],
		buffer[AUDIO_IN_SAMPLE_CNT - 1]);

	return 0;

}