#ifndef __IFFT_DMA_H__
#define __IFFT_DMA_H__

#include <stdint.h>

void claim_dac_samples(void *buf, uint32_t len);
int ifft_init(void);

#endif /* __FFT_DMA_H__ */