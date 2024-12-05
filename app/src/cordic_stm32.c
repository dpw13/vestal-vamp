
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
	uint8_t mem_burst;
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

	const struct cordic_cmd *cmd;
};

/* Returns the number of arguments for each function. See CORDIC documentation
 * for the associated STM32.
 */
static inline int cordic_arg_count(enum cordic_func func)
{
	switch (func) {
	case CORDIC_FUNC_COSINE:
	case CORDIC_FUNC_SINE:
	case CORDIC_FUNC_PHASE:
	case CORDIC_FUNC_MODULUS:
		return 2;
	case CORDIC_FUNC_ARCTANGENT:
	case CORDIC_FUNC_HCOSINE:
	case CORDIC_FUNC_HSINE:
	case CORDIC_FUNC_HARCTANGENT:
	case CORDIC_FUNC_NATURALLOG:
	case CORDIC_FUNC_SQUAREROOT:
		return 1;
	}

	LOG_ERR("Invalid CORDIC func");
	return 0;
}

/* Returns the number of results for each function. See CORDIC documentation
 * for the associated STM32.
 */
static inline int cordic_res_count(enum cordic_func func)
{
	switch (func) {
	case CORDIC_FUNC_COSINE:
	case CORDIC_FUNC_SINE:
	case CORDIC_FUNC_PHASE:
	case CORDIC_FUNC_MODULUS:
	case CORDIC_FUNC_HCOSINE:
	case CORDIC_FUNC_HSINE:
		return 2;
	case CORDIC_FUNC_ARCTANGENT:
	case CORDIC_FUNC_HARCTANGENT:
	case CORDIC_FUNC_NATURALLOG:
	case CORDIC_FUNC_SQUAREROOT:
		return 1;
	}

	LOG_ERR("Invalid CORDIC func");
	return 0;
}

static inline int cordic_arg_width(const struct cordic_cmd *cmd)
{
	if (cmd->arg_format == CORDIC_FORMAT_32) {
		return 2 * cordic_arg_count(cmd->func);
	}
	return cordic_arg_count(cmd->func);
}

static inline int cordic_res_width(const struct cordic_cmd *cmd)
{
	if (cmd->arg_format == CORDIC_FORMAT_32) {
		return 2 * cordic_res_count(cmd->func);
	}
	return cordic_res_count(cmd->func);
}

static inline void cordic_config(CORDIC_TypeDef *cordic, const struct cordic_cmd *cmd)
{
	LL_CORDIC_Config(cordic, (uint32_t)cmd->func << CORDIC_CSR_FUNC_Pos,
			 (uint32_t)cmd->iterations << CORDIC_CSR_PRECISION_Pos,
			 (uint32_t)cmd->lshift << CORDIC_CSR_SCALE_Pos,
			 cordic_arg_width(cmd) == 4 ? LL_CORDIC_NBWRITE_2 : LL_CORDIC_NBWRITE_1,
			 cordic_res_width(cmd) == 4 ? LL_CORDIC_NBREAD_2 : LL_CORDIC_NBREAD_1,
			 cmd->arg_format == CORDIC_FORMAT_16 ? LL_CORDIC_INSIZE_16BITS
							     : LL_CORDIC_INSIZE_32BITS,
			 cmd->res_format == CORDIC_FORMAT_16 ? LL_CORDIC_OUTSIZE_16BITS
							     : LL_CORDIC_OUTSIZE_32BITS);
}

static void dma_tx_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	/* user_data directly holds the cordic device */
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)user_data;

	if (channel == data->dma_tx.channel) {
		if (status < 0) {
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
		if (data->cmd->callback) {
			data->cmd->callback(dev, status);
		}
		if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			LL_CORDIC_DisableDMAReq_RD(data->cordic);
		}
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
	}
}

static int cordic_stm32_dma_start(const struct device *dev, const struct cordic_cmd *cmd,
				  bool is_tx)
{
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = is_tx ? &data->dma_tx : &data->dma_rx;

	blk_cfg = &dma->dma_blk_cfg;

	if (is_tx) {
		blk_cfg->block_size = cmd->count * cordic_arg_width(data->cmd);

		/* memory to peripheral */
		blk_cfg->source_address = (uint32_t)cmd->buffer_arg;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma->dma_cfg.source_data_size = 4;
		dma->dma_cfg.source_burst_length = dma->mem_burst;

		blk_cfg->dest_address = (uint32_t)&data->cordic->WDATA;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		/* Perform 16-bit writes to the CORDIC block only if there's
		 * only one 16-bit argument per operation.
		 */
		dma->dma_cfg.dest_data_size = cordic_arg_width(data->cmd) == 1 ? 2 : 4;
		/* Only perform a single write per write request */
		dma->dma_cfg.dest_burst_length = 1;
	} else {
		blk_cfg->block_size = cmd->count * cordic_res_width(data->cmd);

		/* peripheral to memory */
		blk_cfg->source_address = (uint32_t)&data->cordic->RDATA;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		/* Same computation on the read side */
		dma->dma_cfg.source_data_size = cordic_res_width(data->cmd) == 1 ? 2 : 4;
		/* Only perform a single read per read request */
		dma->dma_cfg.source_burst_length = 1;

		blk_cfg->dest_address = (uint32_t)cmd->buffer_res;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma->dma_cfg.dest_data_size = 4;
		dma->dma_cfg.dest_burst_length = dma->mem_burst;
	}
	blk_cfg->source_reload_en = 0;
	blk_cfg->dest_reload_en = 0;

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

int cordic_stm32_start(const struct device *dev, const struct cordic_cmd *cmd)
{
	struct cordic_stm32_data *data = (struct cordic_stm32_data *)dev->data;

	data->cmd = cmd;

	cordic_stm32_dma_start(dev, cmd, false);
	cordic_stm32_dma_start(dev, cmd, true);

	/* Enable DMA read and write requests */
	LL_CORDIC_EnableDMAReq_WR(data->cordic);
	LL_CORDIC_EnableDMAReq_RD(data->cordic);

	LOG_DBG("CORDIC started");
	return 0;
}

static int cordic_stm32_init(const struct device *dev)
{
	const struct cordic_stm32_config *cfg = (const struct cordic_stm32_config *)dev->config;
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

	LOG_DBG("%s: init complete", dev->name);

	return 0;
}

#define CORDIC_DMA_CHANNEL_INIT(index, src_dev, dest_dev, name)                                    \
	.dma_##name = {                                                                            \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, name)),                  \
		.channel = DT_INST_DMAS_CELL_BY_NAME(index, name, channel),                        \
		.mem_burst = 4,                                                                    \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = STM32_DMA_SLOT(index, name, slot),                     \
				.channel_direction = STM32_DMA_CONFIG_DIRECTION(                   \
					STM32_DMA_CHANNEL_CONFIG(index, name)),                    \
				.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(        \
					STM32_DMA_CHANNEL_CONFIG(index, name)),                    \
				.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(         \
					STM32_DMA_CHANNEL_CONFIG(index, name)),                    \
				.channel_priority = STM32_DMA_CONFIG_PRIORITY(                     \
					STM32_DMA_CHANNEL_CONFIG(index, name)),                    \
				.dma_callback = dma_##name##_callback,                             \
				.block_count = 1,                                                  \
			},                                                                         \
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
		CORDIC_DMA_CHANNEL_INIT(id, MEMORY, PERIPHERAL, tx),                               \
		CORDIC_DMA_CHANNEL_INIT(id, PERIPHERAL, MEMORY, rx),                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &cordic_stm32_init, NULL, &cordic_stm32_data_##id,               \
			      &cordic_stm32_cfg_##id, POST_KERNEL, CONFIG_DMA_INIT_PRIORITY, NULL)

DT_INST_FOREACH_STATUS_OKAY(STM32_CORDIC_INIT)
