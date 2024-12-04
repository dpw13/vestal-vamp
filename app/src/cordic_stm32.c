
#define DT_DRV_COMPAT st_stm32_cordic

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
#include <stm32h7xx_ll_cordic.h>
#include "cordic_stm32.h"

LOG_MODULE_REGISTER(cordic_stm32, LOG_LEVEL_INF);

struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
};

struct cordic_stm32_config {
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	/* reset controller device configuration*/
	const struct reset_dt_spec reset;
};

struct cordic_stm32_data {
	CORDIC_TypeDef *cordic;

	/* For now only sample input DMA is implemented. */
	struct stream dma_tx;
	struct stream dma_rx;
	volatile int dma_error;

	void *buffer_tx;
	void *buffer_rx;
	size_t buffer_len;

	cordic_dma_callback callback_tx;
	cordic_dma_callback callback_rx;
};

static void dma_tx_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	/* user_data directly holds the cordic device */
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)user_data;

	if (channel == data->dma_tx.channel) {
		if (data->callback_tx) {
			data->callback_tx(dev, status);
		}
		if (status >= 0) {
			LOG_DBG("status %d at %d samples", status, data->buffer_len);
		} else if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			LL_CORDIC_DisableDMAReq_WR(data->cordic);
		}
		dma_stop(data->dma_tx.dma_dev, data->dma_tx.channel);
	}
}

static void dma_rx_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	/* user_data directly holds the cordic device */
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)user_data;

	if (channel == data->dma_rx.channel) {
		if (data->callback_rx) {
			data->callback_rx(dev, status);
		}
		if (status >= 0) {
			LOG_DBG("status %d at %d samples", status, data->buffer_len);
		} else if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			LL_CORDIC_DisableDMAReq_RD(data->cordic);
		}
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
	}
}

static int cordic_stm32_dma_start(const struct device *dev, void *buffer, size_t buffer_len,
				  bool is_tx)
{
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = is_tx ? &data->dma_tx : &data->dma_rx;

	blk_cfg = &dma->dma_blk_cfg;

	/* prepare the block */
	blk_cfg->block_size = buffer_len;

	/* Source and destination */

	/* Use 16-bit access of LSB (add 2 for MSB) */
	if (is_tx) {
		/* memory to peripheral */
		blk_cfg->source_address = (uint32_t)buffer;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		blk_cfg->dest_address = (uint32_t)&data->cordic->WDATA;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* peripheral to memory */
		blk_cfg->dest_address = (uint32_t)buffer;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		blk_cfg->source_address = (uint32_t)&data->cordic->RDATA;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}
	blk_cfg->source_reload_en = 1;
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

	ret = dma_config(dma->dma_dev, dma->channel, &dma->dma_cfg);
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA: %d", ret);
		return ret;
	}

	data->dma_error = 0;
	ret = dma_start(dma->dma_dev, dma->channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	LOG_DBG("DMA started: %d B to %p", blk_cfg->block_size, (void *)blk_cfg->dest_address);

	return ret;
}

int cordic_stm32_start(const struct device *dev, void *buffer_tx, void *buffer_rx,
		       size_t buffer_len, cordic_dma_callback cb)
{
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)dev->data;

	data->buffer_tx = buffer_tx;
	data->buffer_rx = buffer_rx;
	data->buffer_len = buffer_len;
	data->callback_tx = cb;
	data->callback_rx = cb;
	cordic_stm32_dma_start(dev, buffer_rx, buffer_len, false);
	cordic_stm32_dma_start(dev, buffer_tx, buffer_len, true);

	/* Enable interrupts */
	LL_CORDIC_EnableIT(data->cordic);

	/* Start DMA and enable DMA write request */
	LL_CORDIC_EnableDMAReq_WR(data->cordic);

	LOG_DBG("CORDIC started");
	return 0;
}

static int cordic_stm32_init(const struct device *dev)
{
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)dev->data;
	const struct cordic_stm32_config *cfg = (const struct cordic_stm32_config *)dev->config;
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
		LOG_ERR("Could not enable CORDIC clock");
		return err;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("Reset controller device not ready");
		return -ENODEV;
	}

	err = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
	if (K_ERR_ARM_BUS_FP_LAZY_STATE_PRESERVATION != 0) {
		LOG_ERR("CORDIC reset failed");
		return err;
	}

	/*
		uint32_t Precision,
		uint32_t Scale,
		uint32_t NbWrite,
		uint32_t NbRead,
		uint32_t InSize,
		uint32_t OutSize
	*/
	//ll_ret = LL_CORDIC_Config(data->cordic);
	if (ll_ret != SUCCESS) {
		LOG_ERR("Failed to initialize CORDIC filter channel");
		return (int)ll_ret;
	}

	LOG_DBG("%s: init complete", dev->name);

	return 0;
}

#define CORDIC_DMA_CHANNEL_INIT(index, src_dev, dest_dev, name)                                    \
	.dma##name = {                                                                             \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, name)),                  \
		.channel = DT_INST_DMAS_CELL_BY_NAME(index, name, channel),                        \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = STM32_DMA_SLOT_BY_NAME(index, name, slot),             \
				.channel_direction = STM32_DMA_CONFIG_DIRECTION(                   \
					STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),            \
				.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(        \
					STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),            \
				.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(         \
					STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),            \
				.source_burst_length = 1, /* SINGLE transfer */                    \
				.dest_burst_length = 1,   /* SINGLE transfer */                    \
				.channel_priority = STM32_DMA_CONFIG_PRIORITY(                     \
					STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),            \
				.dma_callback = dma_##name##_callback,                             \
				.complete_callback_en = 1, /* Callback at each block */            \
				.block_count = 1,                                                  \
			},                                                                         \
		.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(                       \
			STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),                            \
		.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(                      \
			STM32_DMA_CHANNEL_CONFIG_BY_NAME(index, name)),                            \
	}

#define STM32_CORDIC_INIT(id)                                                                      \
                                                                                                   \
	static const struct stm32_pclken pclken_##id[] = STM32_DT_INST_CLOCKS(id);                 \
                                                                                                   \
	static const struct cordic_stm32_config cordic_stm32_cfg_##id = {                          \
		.pclken = pclken_##id,                                                             \
		.pclk_len = DT_INST_NUM_CLOCKS(id),                                                \
		.reset = RESET_DT_SPEC_INST_GET(id),                                               \
	};                                                                                         \
                                                                                                   \
	static struct cordic_stm32_data cordic_stm32_data_##id = {                                 \
		.cordic = (CORDIC_TypeDef *)DT_INST_REG_ADDR(id),                                  \
		.op = (enum cordic_op)DT_ENUM_IDX_OR(id, operation, 0),                            \
		.x1_wm = DT_INST_PROP_OR(id, x1_wm, 1),                                            \
		.x1_size = DT_INST_PROP_OR(id, x1_size, 4),                                        \
		.x2_size = DT_INST_PROP_OR(id, x2_size, 248),                                      \
		.y_wm = DT_INST_PROP_OR(id, y_wm, 1),                                              \
		.y_size = DT_INST_PROP_OR(id, y_size, 4),                                          \
		.lshift = DT_INST_PROP_OR(id, lshift, 0),                                          \
		.feedback_coeffs = DT_INST_PROP_OR(id, feedback_coeffs, 0),                        \
		.continuous = true,                                                                \
		.clip = DT_INST_PROP_OR(id, clip, 1),                                              \
		CORDIC_DMA_CHANNEL_INIT(id, MEMORY, PERIPHERAL, tx),                               \
		CORDIC_DMA_CHANNEL_INIT(id, PERIPHERAL, MEMORY, rx),                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &cordic_stm32_init, NULL, &cordic_stm32_data_##id,               \
			      &cordic_stm32_cfg_##id, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY, NULL)

DT_INST_FOREACH_STATUS_OKAY(STM32_CORDIC_INIT)
