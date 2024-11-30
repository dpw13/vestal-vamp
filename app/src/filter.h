#ifndef __FILTER_IN_H__
#define __FILTER_IN_H__

#include <stdint.h>
#include "audio.h"

int filter_start(void);
uint32_t filter_get_dac_dma_addr(void);


#endif /* __FILTER_IN_H__ */