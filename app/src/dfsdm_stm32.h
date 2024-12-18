#ifndef __DFSDM_STM32_H__
#define __DFSDM_STM32_H__

#include <zephyr/kernel.h>

typedef int (*dfsdm_dma_callback)(const struct device *dev, int status, int32_t min, int32_t max);

int dfsdm_stm32_start(const struct device *dev, void *buffer, size_t buffer_len,
		      dfsdm_dma_callback cb);

#endif /* __DFSDM_STM32_H__ */