#ifndef __DMA_H__
#define __DMA_H__

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>

/**
 * Helper functions and structure for direct DMA device control.
 */

struct dma_def {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};

#define DT_DMA_DEF_BY_NAME(inst, name, burst, callback)                                            \
	.dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(inst, name)),                                \
	.channel = DT_DMAS_CELL_BY_NAME(inst, name, channel),                                      \
	.dma_cfg = {                                                                               \
		.dma_slot = DT_DMAS_CELL_BY_NAME(inst, name, slot),                                \
		.channel_direction = STM32_DMA_CONFIG_DIRECTION(                                   \
			DT_DMAS_CELL_BY_NAME(inst, name, channel_config)),                         \
		.source_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(                             \
			DT_DMAS_CELL_BY_NAME(inst, name, channel_config)),                         \
		.dest_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(                               \
			DT_DMAS_CELL_BY_NAME(inst, name, channel_config)),                         \
		.source_burst_length = burst,                                                      \
		.dest_burst_length = burst,                                                        \
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(                                     \
			DT_DMAS_CELL_BY_NAME(inst, name, channel_config)),                         \
		.dma_callback = callback,                                                          \
		.complete_callback_en = 1, /* Callback at each block */                            \
		.block_count = 1,                                                                  \
	}

#endif /* __DMA_H__ */