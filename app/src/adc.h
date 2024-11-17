#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

/* Internal */
#define SAMPLE_COUNT 16
#define ADC_RESOLUTION 16

int adc_init(void);
int adc_start(void);
int adc_stats(void);

#endif /* __ADC_H__ */