#ifndef __FILTER_IN_H__
#define __FILTER_IN_H__

#include <stdint.h>
#include "audio.h"

#define FMAC_SINC_ORDER 3

int filter_start(void);
int filter_stats(void);

int32_t filter_get_log_amp(void);
uint32_t filter_get_dac_dma_addr(void);

#endif /* __FILTER_IN_H__ */