
#define DT_DRV_COMPAT st_stm32_fmac

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/toolchain.h>

#include <stm32_ll_dma.h>
#include <stm32h7xx_ll_fmac.h>
#include "fmac_stm32.h"

LOG_MODULE_REGISTER(fmac_stm32, LOG_LEVEL_INF);

struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
};

enum fmac_op {
	FMAC_OP_CONV = 0,
	FMAC_OP_IIR = 1,
};

struct fmac_stm32_config {
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	/* reset controller device configuration*/
	const struct reset_dt_spec reset;
};

struct fmac_stm32_data {
	FMAC_TypeDef *fmac;
	enum fmac_op op;
	uint8_t x1_wm;
	uint8_t x1_size;
	uint8_t x2_size;
	uint8_t y_wm;
	uint8_t y_size;
	uint8_t lshift;

	uint8_t feedback_coeffs;

	/* For now only sample input DMA is implemented. */
	struct stream dma;
	volatile int dma_error;

	uint8_t *buffer;
	size_t buffer_len;
	bool continuous;
	bool clip;

	fmac_dma_callback callback;
};

static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	/* user_data directly holds the fmac device */
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)user_data;

	if (channel == data->dma.channel) {
		if (status >= 0) {
			LOG_DBG("status %d at %d samples", status, data->buffer_len);
			data->callback(dev, status);
			if (!data->continuous) {
				/* Stop the DMA engine, only to start it again when the callback
				 * returns FMAC_ACTION_REPEAT or FMAC_ACTION_CONTINUE, or the number
				 * of samples haven't been reached Starting the DMA engine is done
				 * within fmac_context_start_sampling
				 */
				dma_stop(data->dma.dma_dev, data->dma.channel);
			}
		} else if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			LL_FMAC_DisableDMAReq_WRITE(data->fmac);
			dma_stop(data->dma.dma_dev, data->dma.channel);
			data->callback(dev, status);
		}
	}
}

static int fmac_stm32_dma_start(const struct device *dev, void *buffer, size_t buffer_len)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = &data->dma;

	data->buffer = (uint8_t *)buffer;
	data->buffer_len = buffer_len;

	blk_cfg = &dma->dma_blk_cfg;

	/* prepare the block */
	blk_cfg->block_size = buffer_len;

	/* Source and destination */

	/* Use 16-bit access of LSB (add 2 for MSB) */
	blk_cfg->source_address = (uint32_t)buffer;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->source_reload_en = 1;

	blk_cfg->dest_address = (uint32_t)&data->fmac->WDATA;
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->dest_reload_en = 1;

	/* Manually set the FIFO threshold to 1/4 because the
	 * dmamux DTS entry does not contain fifo threshold
	 */
	blk_cfg->fifo_mode_control = 0;

	blk_cfg->next_block = NULL;

	/* direction is given by the DT */
	dma->dma_cfg.block_count = 1;
	dma->dma_cfg.complete_callback_en = 1; /* Callback at each block */

	dma->dma_cfg.head_block = blk_cfg;
	dma->dma_cfg.user_data = data;
	dma->dma_cfg.cyclic = data->continuous;

	ret = dma_config(data->dma.dma_dev, data->dma.channel, &dma->dma_cfg);
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA: %d", ret);
		return ret;
	}

	data->dma_error = 0;
	ret = dma_start(data->dma.dma_dev, data->dma.channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	LOG_DBG("DMA started: %d B to %p", blk_cfg->block_size, (void *)blk_cfg->dest_address);

	return ret;
}

int fmac_stm32_start(const struct device *dev, void *buffer, size_t buffer_len,
		     fmac_dma_callback cb)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;

	data->buffer = (uint8_t *)buffer;
	data->buffer_len = buffer_len;
	data->callback = cb;
	fmac_stm32_dma_start(dev, buffer, buffer_len);

	/* Enable overflow and underflow interrupts */
	LL_FMAC_EnableIT_UNFL(data->fmac);
	LL_FMAC_EnableIT_OVFL(data->fmac);

	/* Start DMA and enable DMA write request */
	dma_start(data->dma.dma_dev, data->dma.channel);
	LL_FMAC_EnableDMAReq_WRITE(data->fmac);

	/* Start processing */
	if (LL_FMAC_IsEnabledStart(data->fmac)) {
		LOG_ERR("FMAC started unexpectedly!");
	}
	if (data->op == FMAC_OP_CONV) {
		LL_FMAC_ConfigFunc(data->fmac, LL_FMAC_PROCESSING_START, LL_FMAC_FUNC_CONVO_FIR,
				   data->x2_size, 0, data->lshift);
	} else {
		LL_FMAC_ConfigFunc(
			data->fmac, LL_FMAC_PROCESSING_START, LL_FMAC_FUNC_IIR_DIRECT_FORM_1,
			data->x2_size - data->feedback_coeffs, data->feedback_coeffs, data->lshift);
	}

	LOG_DBG("FMAC started");
	while (!LL_FMAC_IsEnabledStart(data->fmac))
		;

	return 0;
}

static int fmac_stm32_load_buf(const struct device *dev, uint32_t which, uint16_t *samples,
			       size_t sample_count)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;

	LL_FMAC_ConfigFunc(data->fmac, LL_FMAC_PROCESSING_START, which, sample_count, 0, 0);
	while (!LL_FMAC_IsEnabledStart(data->fmac))
		;
	for (int i = 0; i < sample_count; i++) {
		data->fmac->WDATA = *samples++;
	}
	if (LL_FMAC_IsEnabledStart(data->fmac)) {
		LOG_ERR("FMAC did not stop after loading buffer!");
	} else {
		LOG_DBG("FMAC buffer loaded");
	}
	return 0;
}

static inline uint32_t fmac_stm32_samples_to_wm(uint8_t samples)
{
	switch (samples) {
	case 1:
		return LL_FMAC_WM_0_THRESHOLD_1;
	case 2:
		return LL_FMAC_WM_1_THRESHOLD_2;
	case 4:
		return LL_FMAC_WM_2_THRESHOLD_4;
	case 8:
		return LL_FMAC_WM_3_THRESHOLD_8;
	default:
		LOG_ERR("Invalid watermark value, assuming 2 samples");
		return LL_FMAC_WM_1_THRESHOLD_2;
	}
}

static int fmac_stm32_set_buffers(const struct device *dev)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;
	uint16_t offset = 0;

	LL_FMAC_ConfigX1(data->fmac, fmac_stm32_samples_to_wm(data->x1_wm), offset, data->x1_size);
	offset += data->x1_size;
	LL_FMAC_ConfigX2(data->fmac, offset, data->x2_size);
	offset += data->x2_size;
	LL_FMAC_ConfigY(data->fmac, fmac_stm32_samples_to_wm(data->y_wm), offset, data->y_size);
	offset += data->y_size;

	if (offset > 256) {
		LOG_ERR("Combined buffers greater than 256");
		return -EINVAL;
	}

	return 0;
}

uint32_t fmac_stm32_get_output_reg(const struct device *dev)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;
	return (uint32_t)&data->fmac->RDATA;
}

int fmac_stm32_configure_fir(const struct device *dev, int16_t *coeffs, uint8_t coeff_len)
{
	int ret;
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;
	/* Configure filter function
	 * X1: sample data
	 * X2: filter coeffs
	 * Y: output samples
	 */

	data->x2_size = coeff_len;
	ret = fmac_stm32_set_buffers(dev);
	if (ret) {
		LOG_ERR("Unable to configure buffers");
		return ret;
	}

	/* TODO: Could use DMA here */
	fmac_stm32_load_buf(dev, LL_FMAC_FUNC_LOAD_X2, coeffs, coeff_len);
	if (ret) {
		LOG_ERR("Unable to load coefficients");
		return ret;
	}

	return 0;
}

static int fmac_stm32_init(const struct device *dev)
{
	struct fmac_stm32_data *data = (struct fmac_stm32_data *)dev->data;
	const struct fmac_stm32_config *cfg = (const struct fmac_stm32_config *)dev->config;
	ErrorStatus ll_ret;
	int err;

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* TODO: we don't currently support clock selection */
	err = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			       (clock_control_subsys_t)&cfg->pclken[0]);
	if (err < 0) {
		LOG_ERR("Could not enable FMAC clock");
		return err;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("Reset controller device not ready");
		return -ENODEV;
	}

	err = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
	if (err != 0) {
		LOG_ERR("FMAC reset failed");
		return err;
	}

	ll_ret = LL_FMAC_Init(data->fmac);
	if (ll_ret != SUCCESS) {
		LOG_ERR("Failed to initialize FMAC filter channel");
		return (int)ll_ret;
	}

	LOG_DBG("%s: init complete", dev->name);

	return 0;
}

#define FMAC_DMA_CHANNEL_INIT(index, src_dev, dest_dev)                                            \
	.dma = {                                                                                   \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(index, 0)),                      \
		.channel = DT_INST_DMAS_CELL_BY_IDX(index, 0, channel),                            \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = STM32_DMA_SLOT_BY_IDX(index, 0, slot),                 \
				.channel_direction = STM32_DMA_CONFIG_DIRECTION(                   \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(        \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(         \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.source_burst_length = 1, /* SINGLE transfer */                    \
				.dest_burst_length = 1,   /* SINGLE transfer */                    \
				.channel_priority = STM32_DMA_CONFIG_PRIORITY(                     \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.dma_callback = dma_callback,                                      \
				.complete_callback_en = 1, /* Callback at each block */            \
				.block_count = 1,                                                  \
			},                                                                         \
		.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(                       \
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                                \
		.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(                      \
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                                \
	}

#define STM32_FMAC_INIT(id)                                                                        \
                                                                                                   \
	static const struct stm32_pclken pclken_##id[] = STM32_DT_INST_CLOCKS(id);                 \
                                                                                                   \
	static const struct fmac_stm32_config fmac_stm32_cfg_##id = {                              \
		.pclken = pclken_##id,                                                             \
		.pclk_len = DT_INST_NUM_CLOCKS(id),                                                \
		.reset = RESET_DT_SPEC_INST_GET(id),                                               \
	};                                                                                         \
                                                                                                   \
	static struct fmac_stm32_data fmac_stm32_data_##id = {                                     \
		.fmac = (FMAC_TypeDef *)DT_INST_REG_ADDR(id),                                      \
		.op = (enum fmac_op)DT_ENUM_IDX_OR(id, operation, 0),                              \
		.x1_wm = DT_INST_PROP_OR(id, x1_wm, 1),                                            \
		.x1_size = DT_INST_PROP_OR(id, x1_size, 4),                                        \
		.x2_size = DT_INST_PROP_OR(id, x2_size, 248),                                      \
		.y_wm = DT_INST_PROP_OR(id, y_wm, 1),                                              \
		.y_size = DT_INST_PROP_OR(id, y_size, 4),                                          \
		.lshift = DT_INST_PROP_OR(id, lshift, 0),                                          \
		.feedback_coeffs = DT_INST_PROP_OR(id, feedback_coeffs, 0),                        \
		.continuous = true,                                                                \
		.clip = DT_INST_PROP_OR(id, clip, 1),                                              \
		FMAC_DMA_CHANNEL_INIT(id, MEMORY, PERIPHERAL)};                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &fmac_stm32_init, NULL, &fmac_stm32_data_##id,                   \
			      &fmac_stm32_cfg_##id, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY, NULL)

DT_INST_FOREACH_STATUS_OKAY(STM32_FMAC_INIT)
