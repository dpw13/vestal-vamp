#ifndef __ADC_H__
#define __ADC_H__

#define CHANNEL_COUNT 6

extern uint8_t adc_update;
extern uint16_t adc_res[CHANNEL_COUNT];

void adc_init(void);
void adc_print_state(void);

#endif /* __ADC_H__ */