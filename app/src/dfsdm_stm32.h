#ifndef __DFSDM_STM32_H__
#define __DFSDM_STM32_H__

#include <zephyr/kernel.h>

typedef int (*dfsdm_dma_callback)(const struct device *dev,
					void *buffer,
					size_t sample_count);


int dfsdm_stm32_start(const struct device *dev, void *buffer, size_t buffer_len, dfsdm_dma_callback cb);

#endif /* __DFSDM_STM32_H__ */