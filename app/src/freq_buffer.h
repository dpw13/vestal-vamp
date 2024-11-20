#ifndef __FREQ_BUFFER_H__
#define __FREQ_BUFFER_H__

#include <arm_math.h>
#include <stdint.h>
#include "audio.h"

/**
 * Log2 of the number of frequency buffers available.
 */
#define WINDOW_IDX_BITS 2

/**
 * The number of frequency-domain representations of a single window
 * available for synthesis in memory.
 */
#define WINDOW_COUNT    (1 << WINDOW_IDX_BITS)

#define WINDOW_IDX_MASK     (WINDOW_COUNT - 1)

/**
 * Long-term frequency-domain data
 */
struct polar_freq_data {
        q15_t mag[FFT_SIZE];
        q15_t phase[FFT_SIZE];
};

/* TODO: external PSRAM */
/* Technically this is fixed point 11.5 for FFT size of 1024 */
extern struct polar_freq_data lt_buffer[WINDOW_COUNT];
extern q15_t fft_freq[WINDOW_COUNT*2*FFT_SIZE];
extern uint8_t wr_idx;

/* The window defined for FFTs is also used for IFFTs */
extern q15_t fft_window_func[FFT_SIZE];

/**
 * The type for the fractional index into the frequency buffer.
 */
typedef uint32_t frac_idx_t;

#define FRAC_IDX_UNITY  (frac_idx_t)(1 << (32 - WINDOW_IDX_BITS))

/**
 * Returns a pointer to the beginning of the i'th window.
 */
static inline struct polar_freq_data* get_lt_buf(uint32_t i) {
        return &lt_buffer[i];
}

/**
 * Get the integer part of the window index.
 */
static inline uint32_t idx_ipart(frac_idx_t v) {
        /* Use the top WINDOW_IDX_BITS for the integer part. The remainder
         * are fractional bits. */
        return (v >> (32 - WINDOW_IDX_BITS));
}

/**
 * Get the fractional part of the window index in unsigned 0.32 fxp format.
 */
static inline uint32_t idx_fpart(frac_idx_t v) {
        /* Shift up to generate a 0.32 unsigned fractional value. */
        return v << WINDOW_IDX_BITS;
}

#endif /* __FREQ_BUFFER_H__ */