
#define DT_DRV_COMPAT st_stm32_dfsdm

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
#include <stm32h7xx_hal_dfsdm.h>
#include "dfsdm_stm32.h"

LOG_MODULE_REGISTER(dfsdm_stm32, LOG_LEVEL_INF);

struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
};

struct dfsdm_stm32_config {
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	/* reset controller device configuration*/
	const struct reset_dt_spec reset;
};

struct dfsdm_stm32_data {
	DFSDM_Channel_HandleTypeDef channel;
	DFSDM_Filter_HandleTypeDef filter;
	struct stream dma;
	volatile int dma_error;

	uint8_t *buffer;
	size_t buffer_len;
	bool continuous;

	dfsdm_dma_callback callback;
};

static void dma_callback(const struct device *dev, void *user_data,
			 uint32_t channel, int status)
{
	/* user_data directly holds the dfsdm device */
	struct dfsdm_stm32_data *data = user_data;

	LOG_DBG("dma callback");

	if (channel == data->dma.channel) {
		if (status >= 0) {
			if (data->continuous) {
				/* Half buffer complete in cyclic mode */
				data->buffer_len += data->buffer_len/2;
				data->buffer += data->buffer_len/2;
			} else {
				/* Full buffer complete */
				data->buffer_len += data->buffer_len/2;
				data->buffer += data->buffer_len;
			}
			LOG_DBG("status %d at %d samples", status, data->buffer_len);
			if (data->continuous) {
				/* In continuous mode, we simply continue to transfer data and call
				* the callback. No need to stop/restart the DMA engine or even tell
				* the dfsdm_context subsystem that the buffer is complete.
				*/
				data->callback(dev, data->buffer, data->buffer_len);
			} else {
				/* Stop the DMA engine, only to start it again when the callback returns
				* DFSDM_ACTION_REPEAT or DFSDM_ACTION_CONTINUE, or the number of samples
				* haven't been reached Starting the DMA engine is done
				* within dfsdm_context_start_sampling
				*/
				dma_stop(data->dma.dma_dev, data->dma.channel);
				/* No need to invalidate the cache because it's assumed that
				* the address is in a non-cacheable SRAM region.
				*/
				data->callback(dev, data->buffer, data->buffer_len);
			}
		} else if (status < 0) {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			HAL_DFSDM_FilterRegularStop(&data->filter);
			dma_stop(data->dma.dma_dev, data->dma.channel);
			data->callback(dev, data->buffer, data->buffer_len);
		}

		if (data->filter.State == HAL_DFSDM_FILTER_STATE_ERROR) {
			LOG_ERR("Filter reported error state: %d", data->filter.ErrorCode);
			HAL_DFSDM_FilterRegularStop(&data->filter);
			dma_stop(data->dma.dma_dev, data->dma.channel);
		}
	}
}

static int dfsdm_stm32_dma_start(const struct device *dev,
			       void *buffer, size_t buffer_len)
{
	struct dfsdm_stm32_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = &data->dma;

	data->buffer = buffer;
	data->buffer_len = buffer_len;

	blk_cfg = &dma->dma_blk_cfg;

	/* prepare the block */
	blk_cfg->block_size = buffer_len;

	/* Source and destination */

	/* Use 16-bit access of LSB (add 2 for MSB) */
	blk_cfg->source_address = (uint32_t)data->filter.Instance->FLTJDATAR;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->source_reload_en = 1;

	blk_cfg->dest_address = (uint32_t)buffer;
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
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

	ret = dma_config(data->dma.dma_dev, data->dma.channel,
			 &dma->dma_cfg);
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

int dfsdm_stm32_start(const struct device *dev, void *buffer, size_t buffer_len, dfsdm_dma_callback cb) {
	struct dfsdm_stm32_data *data = dev->data;
	HAL_StatusTypeDef ret;

	data->buffer = buffer;
	data->buffer_len = buffer_len;
	data->callback = cb;
	dfsdm_stm32_dma_start(dev, buffer, buffer_len);

	/* The next step is typically HAL_DFSDM_FilterRegularStart_DMA, but this just starts
	 * the associated DMA and then runs DFSDM_RegConvStart(). */
	dma_start(data->dma.dma_dev, data->dma.channel);
	ret = HAL_DFSDM_FilterRegularStart(&data->filter);

	return 0;
}

static int dfsdm_stm32_init(const struct device *dev)
{
	struct dfsdm_stm32_data *data = dev->data;
	const struct dfsdm_stm32_config *cfg = dev->config;
	HAL_StatusTypeDef ret;
	int err;

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* TODO: we don't currently support clock selection */
	err = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			       (clock_control_subsys_t) &cfg->pclken[0]);
	if (err < 0) {
		LOG_ERR("Could not enable DFSDM clock");
		return err;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("Reset controller device not ready");
		return -ENODEV;
	}

	ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
	if (ret != 0) {
		LOG_ERR("DFSDM reset failed");
		return ret;
	}

	ret = HAL_DFSDM_ChannelInit(&data->channel);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize DFSDM filter channel");
		return ret;
	}

	ret = HAL_DFSDM_FilterInit(&data->filter);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize DFSDM filter");
		return ret;
	}

	ret = HAL_DFSDM_FilterConfigRegChannel(&data->filter, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to configure DFSDM filter");
		return ret;
	}

	return 0;
}

#define DFSDM_DMA_CHANNEL_INIT(index, src_dev, dest_dev)					\
	.dma = {									\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(index, 0)),		\
		.channel = DT_INST_DMAS_CELL_BY_IDX(index, 0, channel),			\
		.dma_cfg = {								\
			.dma_slot = STM32_DMA_SLOT_BY_IDX(index, 0, slot),		\
			.channel_direction = STM32_DMA_CONFIG_DIRECTION(		\
				STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),		\
			.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(	\
				STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),		\
			.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(	\
				STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),		\
			.source_burst_length = 1,       /* SINGLE transfer */		\
			.dest_burst_length = 4,         /* Burst transfer to mem */	\
			.channel_priority = STM32_DMA_CONFIG_PRIORITY(			\
				STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),		\
			.dma_callback = dma_callback,					\
			.complete_callback_en = 1, /* Callback at each block */		\
			.block_count = 1,						\
		},									\
		.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(		\
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),			\
		.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(		\
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),			\
	}

#define STM32_DFSDM_INIT(id)							\
										\
static const struct stm32_pclken pclken_##id[] =				\
					       STM32_DT_INST_CLOCKS(id);	\
										\
static const struct dfsdm_stm32_config dfsdm_stm32_cfg_##id = {			\
	.pclken = pclken_##id,							\
	.pclk_len = DT_INST_NUM_CLOCKS(id),					\
	.reset = RESET_DT_SPEC_INST_GET(id),					\
};										\
										\
static struct dfsdm_stm32_data dfsdm_stm32_data_##id = {			\
	.channel = {								\
		.Instance = (DFSDM_Channel_TypeDef  *) DT_INST_REG_ADDR_BY_NAME(id, channel), \
		.Init = {							\
			.Input = {						\
				.Multiplexer = DFSDM_CHANNEL_ADC_OUTPUT,	\
				.DataPacking = DFSDM_CHANNEL_STANDARD_MODE,     \
				.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS,	\
			},							\
			.Offset = 0,						\
			.RightBitShift = 4,					\
		},								\
	},									\
	.filter = {								\
		.Instance = (DFSDM_Filter_TypeDef  *) DT_INST_REG_ADDR_BY_NAME(id, filter), \
		.Init = {							\
			.RegularParam = {					\
				.Trigger = DFSDM_FILTER_SW_TRIGGER,		\
				.FastMode = ENABLE,				\
				.DmaMode = ENABLE,				\
			},							\
			.FilterParam = {					\
				.IntOversampling = 1, /* Integrator bypass */	\
				.Oversampling = 16,				\
				.SincOrder = DFSDM_FILTER_SINC5_ORDER, /* Shrug? */ \
			},							\
		},								\
	},									\
	.continuous = true,							\
	DFSDM_DMA_CHANNEL_INIT(id, PERIPHERAL, MEMORY)				\
};										\
										\
DEVICE_DT_INST_DEFINE(id,							\
		    &dfsdm_stm32_init, NULL,					\
		    &dfsdm_stm32_data_##id, &dfsdm_stm32_cfg_##id,		\
		    POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,			\
		    NULL);

DT_INST_FOREACH_STATUS_OKAY(STM32_DFSDM_INIT)
