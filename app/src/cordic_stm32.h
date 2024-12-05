#ifndef __CORDIC_STM32_H__
#define __CORDIC_STM32_H__

#include <zephyr/kernel.h>

enum cordic_func {
	CORDIC_FUNC_COSINE,
	CORDIC_FUNC_SINE,
	CORDIC_FUNC_PHASE,
	CORDIC_FUNC_MODULUS,
	CORDIC_FUNC_ARCTANGENT,
	CORDIC_FUNC_HCOSINE,
	CORDIC_FUNC_HSINE,
	CORDIC_FUNC_HARCTANGENT,
	CORDIC_FUNC_NATURALLOG,
	CORDIC_FUNC_SQUAREROOT,
};

enum cordic_data_format {
	CORDIC_FORMAT_16,
	CORDIC_FORMAT_32,
};

typedef int (*cordic_dma_callback)(const struct device *dev, int status);

struct cordic_cmd {
	/**
	 * The math operation to perform
	 */
	enum cordic_func func;

	/**
	 * The number of iterations to execute. Higher number of iterations
	 * takes longer but produces lower error. Valid values 1-15, with
	 * values of 3-6 recommended for most functions.
	 */
	uint8_t iterations; /* 1-15 */

	/**
	 * The number of bits to left-shift the result. Valid values 1-7.
	 */
	uint8_t lshift;

	/**
	 * The format of the input data to be processed.
	 */
	enum cordic_data_format arg_format;

	/**
	 * The format of the output data.
	 */
	enum cordic_data_format res_format;

	/**
	 * The source (argument) buffer.
	 */
	void *buffer_arg;

	/**
	 * The destination (result) buffer.
	 */
	void *buffer_res;

	/**
	 * The number of computations to perform. The number and size of
	 * the argument and result values depends on the function being
	 * performed and the width of the inputs and outputs.
	 */
	uint32_t count;

	/**
	 * An optional callback to call once the computation is performed and
	 * the destination buffer has been filled.
	 */
	cordic_dma_callback callback;
};

int cordic_stm32_start(const struct device *dev, const struct cordic_cmd *cmd);

#endif /* __CORDIC_STM32_H__ */