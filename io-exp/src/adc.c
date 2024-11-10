#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "adc.h"

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof(array[0]))

uint16_t adc_sample_count;
uint8_t adc_channel_idx;

const uint8_t adc0_channel_list[] = {
    ADC_MUXPOS_AIN6_gc,
    ADC_MUXPOS_AIN7_gc,
    ADC_MUXPOS_AIN8_gc,
    ADC_MUXPOS_AIN9_gc,
    ADC_MUXPOS_AIN10_gc,
    ADC_MUXPOS_AIN11_gc};

#define ADC0_RES_OFFSET 0
#define TOTAL_CHANNELS ARRAY_SIZE(adc0_channel_list)

uint16_t adc_res[TOTAL_CHANNELS];
uint8_t adc_update;

#define LOG2_SAMPLES    4
#define SAMPLE_MULT     (1 << LOG2_SAMPLES)

void adc_print_state(void) {
    printf("ADC update: %d count %u\n", adc_update, adc_sample_count);
    for (uint8_t i=0; i<TOTAL_CHANNELS; i++) {
        printf("  ch%d: raw %04x\n", i, adc_res[i]);
    }
}

ISR(ADC0_RESRDY_vect) {
    uint16_t res = ADC0.RES;
    uint8_t idx = adc_channel_idx;
    uint8_t nx_idx = adc_channel_idx + 1;

    ADC0.INTFLAGS = ADC0.INTFLAGS;

    /* Select next channel */
    if (nx_idx == ARRAY_SIZE(adc0_channel_list)) {
        nx_idx = 0;
    }
    ADC0.MUXPOS = adc0_channel_list[nx_idx];
    /* Start next conversion */
    ADC0.COMMAND = ADC_STCONV_bm;

    adc_channel_idx = nx_idx;

    /* Store previous sample */
    idx += ADC0_RES_OFFSET;
    adc_sample_count++;

    /* Store */
    adc_res[idx] = res;
}

static inline void adc_n_init(ADC_t *adc) {
    adc->CTRLA = ADC_RESSEL_10BIT_gc;
    /* Accumulate a single sample */
    adc->CTRLB = LOG2_SAMPLES << ADC_SAMPNUM_gp;
    adc->CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV64_gc;
    /* 32 clocks of mux settling, 0 clocks delay */
    adc->CTRLD = ADC_INITDLY_DLY32_gc | ADC_ASDV_ASVOFF_gc;
    adc->CTRLE = ADC_WINCM_NONE_gc;
    /* 8 clocks precharge */
    adc->SAMPCTRL = 0x06;
    adc->INTCTRL = ADC_RESRDY_bm;
}

void adc_init(void) {
    adc_n_init(&ADC0);

    ADC0.CTRLA |= ADC_ENABLE_bm;

    ADC0.MUXPOS = adc0_channel_list[0];

    /* Start conversion. */
    ADC0.COMMAND = ADC_STCONV_bm;
}
