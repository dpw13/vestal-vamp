#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>
#include "audio.h"

/* Internal */
#define SAMPLE_COUNT 16
#define ADC_RESOLUTION 16

/* The amount of data for each callback. This is half the buffer size because
 * we use the half-transfer interrupt used in cyclic mode
 */
#define AUDIO_IN_BLOCK_SIZE	FFT_SIZE

extern audio_raw_t adc_buffer[AUDIO_IN_BLOCK_SIZE*2];

int adc_init(void);
int adc_start(void);
int adc_stats(void);

#endif /* __ADC_H__ */