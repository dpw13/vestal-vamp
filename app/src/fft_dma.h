#ifndef __FFT_DMA_H__
#define __FFT_DMA_H__

#include <stdint.h>

void submit_adc_samples(void *buf, uint32_t len);
int fft_init(void);

#endif /* __FFT_DMA_H__ */