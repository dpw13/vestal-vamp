#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "audio.h"
#include "adc.h"
#include "dma.h"

#include <arm_math.h>
#include <arm_const_structs.h>

LOG_MODULE_REGISTER(fft_dma, LOG_LEVEL_INF);

static uint16_t fft_buffer[FFT_SIZE] __attribute__((__section__("SRAM1")));

static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status);

/* Memory-to-memory burst size */
#define DMA_M2M_BURST	8

/**
 * The memory-to-memory DMA device and configuration for FFT input
 */
static struct dma_def dma_def = {
	DT_DMA_DEF_BY_NAME(DT_PATH(zephyr_user), audio_in_dma, DMA_M2M_BURST, dma_callback),
	.dma_blk_cfg = {
		.block_size = FFT_SIZE * sizeof(int16_t),

		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.source_reload_en = 0,

		.dest_address = (uint32_t)&fft_buffer[0],
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_reload_en = 0,

		/* Manually set the FIFO threshold to 1/4 because the
		* dmamux DTS entry does not contain fifo threshold
		*/
		.fifo_mode_control = 0,
		.next_block = NULL,
	}
};

/**
 * The current sequence index
 */
uint8_t active_seq;

/**
 * DMA parameters from ADC memory buffer to FFT buffer
 */
struct inst_seq {
	uint32_t dma_src;       /*!< Source address for DMA transfer */
	uint32_t dma_dst;       /*!< Destination address for DMA transfer */
	uint32_t dma_len;       /*!< DMA transfer size in bytes */

        /* Whether the FFT is ready to run once the DMA is complete */
	bool fft_ready;
	/* Whether the next DMA operation can be executed immediately */
	bool dma_ready;
};

/**
 * Helper macro for enumerating DMA/FFT sequence steps.
 * 
 * @param quarter Which quarter of the buffer to start DMA at
 * @param frac 1 to transfer a full FFT_SIZE block, 2 to transfer FFT_SIZE/2
 * @param dst_half Which half of a partial block (frac=2) to transfer
 * @param _fft_ready If true, schedule an FFT on the data once transferred
 * @param _dma_ready If true, schedule the next DMA once the FFT has completed
 */
#define SEQ_INST(quarter, frac, dst_half, _fft_ready, _dma_ready) { \
	.dma_src = (uint32_t)&adc_buffer[(quarter)*FFT_SIZE/2], \
	.dma_dst = (uint32_t)&fft_buffer[(dst_half)*FFT_SIZE/2], \
	.dma_len = sizeof(uint16_t)*FFT_SIZE/(frac), \
	.fft_ready = _fft_ready, \
	.dma_ready = _dma_ready, \
}

/**
 * The schedule of DMA transfers and FFT computation to do for audio in. This
 * assumes the ADC DMA delivers one full FFT worth of raw samples each time
 * `fft_input_ready` is called, and that we compute FFTs with 50% overlap so
 * that there are 2 FFTs computed each time `fft_input_ready` is called.
 */
static const struct inst_seq seq[] = {
	SEQ_INST(0, 1, 0, true, false),
	SEQ_INST(1, 1, 0, true, true),
	SEQ_INST(2, 1, 0, true, false),
	SEQ_INST(3, 2, 0, false, true),
	SEQ_INST(0, 2, 1, true, true),
};

static inline void incr_seq(void) {
	if (++active_seq == ARRAY_SIZE(seq))
		active_seq = 0;
}

static inline void xfer_next_buffer(void) {
	uint32_t src = seq[active_seq].dma_src;
	uint32_t dst = seq[active_seq].dma_dst;
	uint32_t len = seq[active_seq].dma_len;
	int ret;

	ret = dma_reload(dma_def.dma_dev, dma_def.channel, 
		src, dst, len);
	if (ret < 0) {
		LOG_ERR("Could not reload DMA: %d", ret);
	}
	LOG_DBG("DMA started: %d B from %p to %p", len, (void *)src, (void *)dst);
}

void fft_input_ready(void *buf, uint32_t len) {
        /* TODO: check that buffers are available and we're at the expected
         * sequence step.
         */
        xfer_next_buffer();
}

/**
 * If the current state indicates that we can immediately start DMA for
 * the next state, do so, then advance buffer window. Call this once all
 * work is completed for this window.
 */
static void window_done(void) {
	bool start_dma = seq[active_seq].dma_ready;
	incr_seq();
	if (start_dma) {
		xfer_next_buffer();
	}
}

/* FFT support structures */
static arm_rfft_instance_q15 S;

static q15_t window[FFT_SIZE] __attribute__((__section__("SRAM1")));;

/* TODO: external SRAM */
/* Technically this is fixed point 11.5 for FFT size of 1024 */
static q15_t fft_freq[2*FFT_SIZE];

/* Work handler to process display updates */
void fft_work_handler(struct k_work *work) {
	LOG_DBG("Window start");
	/* Apply window in-place to ADC samples */
	arm_mult_q15(&window[0], &fft_buffer[0], &fft_buffer[0], FFT_SIZE);
	LOG_DBG("FFT start");
        arm_rfft_q15(&S, (q15_t *)&fft_buffer[0], &fft_freq[0]);
	LOG_DBG("FFT done");
	window_done();
}

/* Define the work handler */
K_WORK_DEFINE(fft_work, fft_work_handler);

static inline void schedule_fft(void) {
        int ret = k_work_submit(&fft_work);
        if (ret < 0) {
            LOG_ERR("Error submitting work: %d", ret);
        }
}

static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status) {
	/* TODO: book-keeping of buffers, check for overrun */
	if (seq[active_seq].fft_ready) {
		schedule_fft();
	} else {
		window_done();
	}
}

/* Copied from arm_float_to_q15 and adapted to a single value */
static inline q15_t arm_float_to_q15_once(float32_t in) {
	return (q15_t) __SSAT((q31_t) (in * 32768.0f), 16);
}

/* Copied from arm_hanning_f32 */
static void arm_hanning_q15(
        q15_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = PI * i * k;
     w = 0.5f * (1.0f - cosf (w));
     pDst[i] = arm_float_to_q15_once(w);
   }
}


int fft_init(void) {
        int ret;

	/* Configure DMA for uncached SRAM4 buffer to cached SRAM1 buffer */
	dma_def.dma_cfg.head_block = &dma_def.dma_blk_cfg;
	ret = dma_config(dma_def.dma_dev, dma_def.channel, &dma_def.dma_cfg);
	if (ret != 0) {
		LOG_ERR("Could not configure DMA: %d", ret);
		return ret;
	}

        /* Initialize FFT vectors */
        arm_status st = arm_rfft_init_1024_q15(&S, 0, 1);
        if (st != ARM_MATH_SUCCESS) {
                LOG_ERR("Could not initialize FFT %d", st);
                return (int)st;
        }

	/* Initialize window */
	arm_hanning_q15(&window[0], FFT_SIZE);

        return 0;
}