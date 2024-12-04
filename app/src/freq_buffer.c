#include "freq_buffer.h"

/* This file only exists to instantiate the necessary shared buffers. */

/* TODO: external SRAM */
/* Technically this is fixed point 11.5 for FFT size of 1024 */
q15_t fft_freq[WINDOW_COUNT * 2 * FFT_SIZE];
uint8_t wr_idx;

/* The long-term data buffer */
struct polar_freq_data lt_buffer[WINDOW_COUNT];
