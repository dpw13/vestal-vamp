#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "adc.h"
#include "dma.h"

LOG_MODULE_REGISTER(audio_adc);

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channel = 
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), audio_in);

/* The amount of data for each callback. This is half the buffer size because
 * we use the half-transfer interrupt used in cyclic mode
 */
#define AUDIO_IN_BLOCK_SIZE	1024
static uint16_t adc_buffer[AUDIO_IN_BLOCK_SIZE*2] __attribute__((__section__("SRAM4")));
static uint16_t fft_buffer[AUDIO_IN_BLOCK_SIZE] __attribute__((__section__("SRAM1")));

uint8_t active_seq;

struct inst_seq {
	/* DMA parameters from ADC memory buffer to FFT buffer */
	uint32_t dma_src;
	uint32_t dma_dst;
	uint32_t dma_len;
	/* Whether the FFT is ready to run once the DMA is complete */
	bool fft_ready;
	/* Whether the next DMA operation can be executed immediately */
	bool dma_ready;
}

#define SEQ_INST(quarter, frac, dst_half, _fft_ready, _dma_ready) { \
	.dma_src = (uint32_t)&adc_buffer[(quarter)*AUDIO_IN_BLOCK_SIZE/2], \
	.dma_dst = (uint32_t)&fft_buffer[(dst_half)*AUDIO_IN_BLOCK_SIZE/2], \
	.dma_len = sizeof(uint16_t)*AUDIO_IN_BLOCK_SIZE/(frac), \
	.fft_ready = _fft_ready, \
	.dma_ready = _dma_ready, \
}

static const seq[] = {
	SEQ_INST(0, 1, 0, true, false),
	SEQ_INST(1, 1, 0, true, true),
	SEQ_INST(2, 1, 0, true, false),
	SEQ_INST(3, 2, 0, false, true),
	SEQ_INST(0, 2, 1, true, true),
};

static inline void incr_seq(void) {
	if (++active_seq == ARRAY_SIZE(seq))
		active_seq = 0;
}

static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status);

#define DMA_M2M_BURST	8
static struct dma_def dma_def = {
	DT_DMA_DEF_BY_NAME(DT_PATH(zephyr_user), audio_in_dma, DMA_M2M_BURST, dma_callback),
	.dma_blk_cfg = {
		.block_size = AUDIO_IN_BLOCK_SIZE * sizeof(int16_t),

		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.source_reload_en = 0,

		.dest_address = (uint32_t)&fft_buffer[0],
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_reload_en = 0,

		/* Manually set the FIFO threshold to 1/4 because the
		* dmamux DTS entry does not contain fifo threshold
		*/
		.fifo_mode_control = 0,
		.next_block = NULL,
	}
};

static inline void xfer_next_buffer(void) {
	uint32_t src = seq[active_seq].dma_src;
	uint32_t dst = seq[active_seq].dma_dst;
	uint32_t len = seq[active_seq].dma_len;
	int ret;

	ret = dma_reload(dma_def.dma_dev, dma_def.channel, 
		src, dst, len);
	if (ret < 0) {
		LOG_ERR("Could not reload DMA: %d", ret);
	}
	LOG_DBG("DMA started: %d B from %p to %p", len, (void *)src, (void *)dst);
}

static void window_done(void) {
	bool start_dma = seq[active_seq].dma_ready;
	incr_seq();
	if (start_dma) {
		xfer_next_buffer();
	}
}

static inline void schedule_fft(void) {
	/* TODO: enqueue FFT work. Increment should occur when FFT is complete */
	LOG_DBG("FFT done");
	window_done();
}

static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status) {
	/* TODO: book-keeping of buffers, check for overrun */
	if (seq[active_seq].fft_ready) {
		schedule_fft();
	} else {
		window_done();
	}
}

static uint32_t sample_count;

static enum adc_action adc_sequence_cb(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index) {
	int ret;

	sample_count += AUDIO_IN_BLOCK_SIZE;

	/* By the time we execute, the previous buffer is already being overwritten. We
	 * can only rely on the fact that the *new* buffer will be stable until the next
	 * callback.
	 */
	//LOG_INF("- %s %d: %"PRId16":%"PRId16" x %"PRId16":%"PRId16,
	//	dev->name, sampling_index,
	//	adc_buffer[8], adc_buffer[AUDIO_IN_BLOCK_SIZE - 8],
	//	adc_buffer[AUDIO_IN_BLOCK_SIZE + 8], adc_buffer[2*AUDIO_IN_BLOCK_SIZE - 8]);
	//LOG_INF("adc_sequence_cb %d: %d", sampling_index, sample_count);

	xfer_next_buffer();

	if (sampling_index > 0) {
		return ADC_ACTION_REPEAT;
	}

	return ADC_ACTION_CONTINUE;
}

static const struct adc_sequence_options opts = {
	.callback = &adc_sequence_cb,
	.block_size = AUDIO_IN_BLOCK_SIZE*2,
	.continuous = true,
};

static struct adc_sequence sequence = {
	.options = &opts,
	.buffer = &adc_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(adc_buffer),
};

int adc_init(void) {
	int ret;

	/* Configure channels individually prior to sampling. */
	ret = adc_is_ready_dt(&adc_channel);
	if (!ret) {
		LOG_ERR("ADC controller device %s not ready", adc_channel.dev->name);
		return ret;
	}

	LOG_INF("ADC %s:%d: %d bits, vref %d, %d ticks, trig %d",
		adc_channel.dev->name, adc_channel.channel_id,
		adc_channel.resolution, adc_channel.vref_mv,
		adc_channel.channel_cfg.acquisition_time, adc_channel.channel_cfg.trig_src);
	ret = adc_channel_setup_dt(&adc_channel);
	if (ret < 0) {
		LOG_ERR("Could not setup channel: %d\n", ret);
		return ret;
	}

	/* Configure DMA for uncached SRAM4 buffer to cached SRAM1 buffer */
	dma_def.dma_cfg.head_block = &dma_def.dma_blk_cfg;
	ret = dma_config(dma_def.dma_dev, dma_def.channel, &dma_def.dma_cfg);
	if (ret != 0) {
		LOG_ERR("Could not configure DMA: %d", ret);
		return ret;
	}

	return 0;
}

int adc_start(void) {
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

int adc_stats(void) {
	LOG_INF("%s: %d samples: %"PRId16":%"PRId16, adc_channel.dev->name,
		sample_count,
		adc_buffer[0],
		adc_buffer[AUDIO_IN_BLOCK_SIZE*2 - 1]);

	return 0;

}
