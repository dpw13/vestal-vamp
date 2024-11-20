#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <stdint.h>

/**
 * The size of the FFT in real samples.
 */
#define FFT_SIZE        1024

/**
 * The natural log base 2 of `INV_OVERLAP`.
 */
#define INV_OVERLAP_BITS 2

/**
 * The inverse of one minus the window overlap amount. If the overlap is 75%, the
 * inverse overlap is 4.
 */
#define INV_OVERLAP     (1 << INV_OVERLAP_BITS)


/**
 * The size of the window overlap in real samples.
 */
#define OVERLAP_SAMPLES	(FFT_SIZE/INV_OVERLAP)

typedef uint16_t audio_raw_t;

#endif /* __AUDIO_H__ */