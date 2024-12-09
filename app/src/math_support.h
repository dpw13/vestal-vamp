#ifndef __MATH_H__
#define __MATH_H__

#include <arm_math.h>
#include <stdint.h>

#define M_PI_F 3.14159265358979323846f
#define M_PI_D 3.14159265358979323846

static inline q31_t mult_q31(q31_t a, q31_t b) {
	return __SSAT(((q63_t)a * b) >> 32, 31);

}

static inline q31_t mult_q15_to_q31(q15_t a, q15_t b) {
	return __SSAT((q31_t)a * b << 1, 31);
}

/* Copied from arm_float_to_q15 and adapted to a single value */
static inline q15_t arm_float_to_q15_once(float32_t in)
{
	return (q15_t)__SSAT((q31_t)(in * 32768.0f), 16);
}

static inline q15_t arm_float_to_q15_once_round(float32_t in)
{
	return (q15_t)__SSAT((q31_t)(in * 32768.0f + 0.5f), 16);
}

/* Copied from arm_float_to_q31 and adapted to a single value */
static inline q31_t arm_float_to_q31_once(float32_t in)
{
	return clip_q63_to_q31((q63_t)(in * 2147483648.0f));
}

static inline q15_t arm_q31_to_q15_once(q31_t in)
{
	return (q15_t)(in >> 16);
}

/* Copied from arm_cmplx_mag_q15. Increments pSrc by 2 */
static inline q15_t arm_cmplx_mag_q15_once_ai(q15_t **pSrc)
{
	q31_t result;
#if defined(ARM_MATH_DSP)
	q31_t in = read_q15x2_ia(pSrc);
	q31_t acc0 = __SMUAD(in, in);

	/* store result in 2.14 format in destination buffer. */
	arm_sqrt_q31((uint32_t)acc0  >> 1 , &result);
#else
	q15_t real = *pSrc++;
	q15_t imag = *pSrc++;
	q31_t acc0 = ((q31_t)real * real);
	q31_t acc1 = ((q31_t)imag * imag);

	/* store result in 2.14 format in destination buffer. */
	arm_sqrt_q31(((uint32_t)acc0 + (uint32_t)acc1) >> 1 , &result);
#endif
	return result >> 16;
}

/* Same as above but does not increment pSrc. */
static inline q15_t arm_cmplx_mag_q15_once_ni(q15_t *pSrc)
{
	q15_t result;
#if defined(ARM_MATH_DSP)
	q31_t in = read_q15x2(pSrc);
	q31_t acc0 = __SMUAD(in, in);

	/* store result in 2.14 format in destination buffer. */
	arm_sqrt_q15((q15_t)(acc0 >> 17), &result);
#else
	q15_t real = *pSrc++;
	q15_t imag = *pSrc++;
	q31_t acc0 = ((q31_t)real * real);
	q31_t acc1 = ((q31_t)imag * imag);

	/* store result in 2.14 format in destination buffer. */
	arm_sqrt_q15((q15_t)(((q63_t)acc0 + acc1) >> 17), &result);
#endif
	return result;
}

static inline q15_t arm_cmplx_mag_sq_q15_once_ni(q15_t *pSrc)
{
	q15_t result;
#if defined(ARM_MATH_DSP)
	q31_t in = read_q15x2(pSrc);
	q31_t acc0 = __SMUAD(in, in);

	/* store result in 2.14 format in destination buffer. */
	return (q15_t)(acc0 >> 17);
#else
	q15_t real = *pSrc++;
	q15_t imag = *pSrc++;
	q31_t acc0 = ((q31_t)real * real);
	q31_t acc1 = ((q31_t)imag * imag);

	/* store result in 2.14 format in destination buffer. */
	return (q15_t)(((q63_t)acc0 + acc1) >> 17);
#endif
	return result;
}

static inline q31_t arm_cmplx_mag_sq_q31_once_ni(q31_t *pSrc)
{
	q31_t real = *pSrc++;
	q31_t imag = *pSrc++;
	q31_t acc0 = (q31_t)(((q63_t)real * real) >> 33);
	q31_t acc1 = (q31_t)(((q63_t)imag * imag) >> 33);

	/* store result in 3.29 format in destination buffer. */
	return acc0 + acc1;
}

/* TODO: I think the q15/q31 ranges don't actually include 1.0... */
#define Q15_ONE ((uint16_t)(1u << 15))
#define Q31_ONE ((uint32_t)(1u << 31))

#endif /* __MATH_H__ */