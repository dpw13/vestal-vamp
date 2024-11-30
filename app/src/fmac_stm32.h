#ifndef __FMAC_STM32_H__
#define __FMAC_STM32_H__

#include <zephyr/kernel.h>

typedef int (*fmac_dma_callback)(const struct device *dev,
					void *buffer,
					size_t sample_count);

int fmac_stm32_configure_fir(const struct device *dev, uint16_t *coeffs, uint8_t coeff_len);
int fmac_stm32_start(const struct device *dev, void *buffer, size_t buffer_len, fmac_dma_callback cb);
uint32_t fmac_stm32_get_output_reg(const struct device *dev);

#endif /* __FMAC_STM32_H__ */