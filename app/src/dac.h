#ifndef __DAC_H__
#define __DAC_H__

#include <stdint.h>
#include "audio.h"

/* Internal */
#define DAC_RESOLUTION 12

int dac_init(void);
int dac_stats(void);

/* The size of the buffer in samples */
#define AUDIO_OUT_SAMPLE_CNT	FFT_SIZE

extern uint16_t dac_buffer[2*AUDIO_OUT_SAMPLE_CNT];

#endif /* __DAC_H__ */