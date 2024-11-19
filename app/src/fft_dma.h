#ifndef __FFT_DMA_H__
#define __FFT_DMA_H__

#include <stdint.h>

void fft_input_ready(void *buf, uint32_t len);
int fft_init(void);

#endif /* __FFT_DMA_H__ */