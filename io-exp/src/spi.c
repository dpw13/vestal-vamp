#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "io.h"
#include "spi.h"

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof(array[0]))

uint8_t tx_byte_count;
uint8_t rx_byte_count;
uint8_t last;

#define ADC_MSG_SIZE    (CHANNEL_COUNT*sizeof(uint16_t))
#define QUAD_MSG_SIZE   (QUAD_COUNT*sizeof(uint16_t))
#define SPI_BUFFER_SIZE (ADC_MSG_SIZE + QUAD_MSG_SIZE)

uint8_t spi_buffer[SPI_BUFFER_SIZE];

/* Master read, device write */
static inline volatile void spi_handle_read(void) {
    if (tx_byte_count < ADC_MSG_SIZE) {
        SPI0.DATA = ((uint8_t *)adc_res)[tx_byte_count++];
    } else if (tx_byte_count < SPI_BUFFER_SIZE) {
        SPI0.DATA = ((uint8_t *)quad_pos)[tx_byte_count++ - ADC_MSG_SIZE];
    }

    if (tx_byte_count == SPI_BUFFER_SIZE) {
        /* Disable further data ready interrupts */
        SPI0.INTFLAGS = SPI_DREIF_bm;
        SPI0.INTCTRL &= ~SPI_DREIE_bm;
    }
}

/* Master write, device read */
static inline volatile void spi_handle_write(void) {
    if (rx_byte_count < SPI_BUFFER_SIZE) {
        spi_buffer[rx_byte_count++] = SPI0.DATA;
    } else {
        /* Read and discard. This ensures that the receive buffer
         * is emptied; simply clearing the interrupt leaves the
         * data in the buffer for the next transfer.
         */
        (void)SPI0.DATA;
    }
}

typedef struct {
    uint8_t start;
    uint8_t isr;
    uint8_t rxc;
    uint8_t txc;
    uint8_t dre;
    uint8_t ssi;
    uint8_t ovf;
} spi_irq_counts_t;

static spi_irq_counts_t spi_irq_counts;

/* Called on falling edge of ~SS */
void spi_start(void) {
    tx_byte_count = 0;
    rx_byte_count = 0;
    /* Write first word */
    spi_handle_read();
    /* Enable DRE interrupt */
    SPI0.INTCTRL |= SPI_DREIE_bm;
    spi_irq_counts.start++;
}

void spi_print_state(void) {
    printf("ISR counts: %02x %02x | %02x %02x %02x | %02x %02x\n",
        spi_irq_counts.start, spi_irq_counts.isr, spi_irq_counts.rxc, spi_irq_counts.txc, spi_irq_counts.dre, spi_irq_counts.ssi, spi_irq_counts.ovf);
}


ISR(SPI0_INT_vect) {
    uint8_t status = SPI0.INTFLAGS & SPI0.INTCTRL;
    //spi_irq_counts.isr++;

    while (status != 0) {
        if (SPI0.INTFLAGS & SPI_RXCIF_bm) {
#if 0
            /* Ignore */
            (void)SPI0.DATA;
            //SPI0.INTFLAGS = SPI_RXCIF_bm;
#else
            //spi_irq_counts.rxc++;
            spi_handle_write();
#endif
        }
        if (status & SPI_DREIF_bm) {
            spi_irq_counts.dre++;
            spi_handle_read();
        }
        /* Handle TXCIF *after* RXCIF and DREIF to reset byte count. */
        if (status & SPI_TXCIF_bm) {
            /* We do not enable interrupts on TXCIF so we should only read this as high
             * when the last receive interrupt fires. */
            spi_irq_counts.txc++;
            SPI0.INTFLAGS = SPI_TXCIF_bm;
        }
        if (status & SPI_BUFOVF_bm) {
            /* We shouldn't ever see an overflow */
            spi_irq_counts.ovf++;
            SPI0.INTFLAGS = SPI_BUFOVF_bm;
        }
        status = SPI0.INTFLAGS & SPI0.INTCTRL;
    }
}

void spi_init(void) {
    /* MSB first, client mode */
    SPI0.CTRLA = 0;
    /* Buffered mode, first MISO byte is a dummy, SPI mode 1 (clk idle low) */
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_MODE_1_gc;

    tx_byte_count = 0;
    rx_byte_count = 0;

    memset(&spi_irq_counts, 0, sizeof(spi_irq_counts_t));

    /* Start with DRE disabled */
    SPI0.INTCTRL = SPI_RXCIE_bm | SPI_IE_bm;

    /* Enable */
    SPI0.CTRLA = SPI_ENABLE_bm;
}