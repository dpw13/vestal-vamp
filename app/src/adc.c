#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "audio.h"
#include "adc.h"
#include "dma.h"
#include "fft_dma.h"

LOG_MODULE_REGISTER(audio_adc, LOG_LEVEL_INF);

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channel =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), audio_in);

static const struct adc_sequence_options opts = {
	.continuous = true,
};

static struct adc_sequence sequence = {
	.options = &opts,
	.buffer = &adc_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(adc_buffer),
};

int adc_init(void)
{
	int ret;

	/* Configure channels individually prior to sampling. */
	ret = adc_is_ready_dt(&adc_channel);
	if (!ret) {
		LOG_ERR("ADC controller device %s not ready", adc_channel.dev->name);
		return ret;
	}

	LOG_INF("ADC %s:%d: %d bits, vref %d, %d ticks, trig %d", adc_channel.dev->name,
		adc_channel.channel_id, adc_channel.resolution, adc_channel.vref_mv,
		adc_channel.channel_cfg.acquisition_time, adc_channel.channel_cfg.trig_src);
	ret = adc_channel_setup_dt(&adc_channel);
	if (ret < 0) {
		LOG_ERR("Could not setup channel: %d\n", ret);
		return ret;
	}

	return 0;
}

int adc_start(void)
{
	int err;

	(void)adc_sequence_init_dt(&adc_channel, &sequence);

	err = adc_read_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("Could not read (%d)", err);
		return err;
	}
	/* This is hacked together such that the read call returns
	 * before data is available, as we're in streaming mode.
	 */

	return 0;
}
