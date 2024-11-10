#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "io.h"
#include "i2c.h"

#define ARRAY_SIZE(array)   (sizeof(array) / sizeof(array[0]))

uint8_t byte_count;

#define ADC_MSG_SIZE    (CHANNEL_COUNT*sizeof(uint16_t))
#define QUAD_MSG_SIZE   (QUAD_COUNT*sizeof(uint16_t))
#define I2C_BUFFER_SIZE (ADC_MSG_SIZE + QUAD_MSG_SIZE)

uint8_t i2c_buffer[I2C_BUFFER_SIZE];

static void i2c_prep_read(void) {
    memcpy(&i2c_buffer[0], &adc_res[0], ADC_MSG_SIZE);
    memcpy(&i2c_buffer[ADC_MSG_SIZE], &quad_pos[0], QUAD_MSG_SIZE);
    adc_update = 0;
    io_update = 0;
}

/* Master read, device write */
static volatile void i2c_handle_read(void) {
    /* TODO, optional: check ACK/NACK from master */
    if (byte_count < I2C_BUFFER_SIZE) {
        TWI0.SDATA = i2c_buffer[byte_count++];
    } else {
        TWI0.SDATA = 0;
    }
    /* TODO: SCTRLB write may not be required when SMEN=1 */
    /* byte_count has been incremented */
    if (byte_count < I2C_BUFFER_SIZE) {
        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
    } else {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    }
}

/* Master write, device read */
static volatile void i2c_handle_write(void) {
    if (byte_count < I2C_BUFFER_SIZE) {
        /* Automatically clears DIF in smart mode */
        i2c_buffer[byte_count++] = TWI0.SDATA;
    }
    /* TODO: SCTRLB write may not be required when SMEN=1 */
    /* byte_count has been incremented */
    if (byte_count < I2C_BUFFER_SIZE) {
        TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
    } else {
        TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
    }
}

uint8_t count_isr;
uint8_t count_dif;
uint8_t count_rd;
uint8_t count_wr;
uint8_t count_apif;
uint8_t count_addr;
uint8_t count_stp;

void i2c_print_state(void) {
    printf("ISR counts: %02x | %02x %02x %02x | %02x %02x %02x\n",
        count_isr, count_dif, count_rd, count_wr, count_apif, count_addr, count_stp);
}


ISR(TWI0_TWIS_vect) {
    uint8_t status = TWI0.SSTATUS;
    count_isr++;

    /* TODO, optional: check COLL bit */
    if (status & TWI_DIF_bm) {
        count_dif++;
        /* Data interrupt */
        if (status & TWI_DIR_bm) {
            /* Master read */
            count_rd++;
            i2c_handle_read();
        } else {
            count_wr++;
            i2c_handle_write();
        }
    } else if (status & TWI_APIF_bm) {
        count_apif++;
        if (status & TWI_AP_bm) {
            count_addr++;
            /* Address interrupt. The device will automatically
             * ACK since the address matches. Fetch the data to
             * transmit.
             */
            byte_count = 0;
            if (status & TWI_DIR_bm) {
                /* Master read */
                i2c_prep_read();
                TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
            } else {
                /* Master write. We still need to clear the interrupt.
                 */
                TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_COMPTRANS_gc;
            }
        } else {
            count_stp++;
            /* STOP interrupt. Writing NOACT to SCTRLB does not appear
             * to clear the interrupt even with smart mode enabled.
             */
            TWI0.SSTATUS = TWI_APIF_bm;
        }
    }
}

void i2c_init(void) {
    /* Set I2C pins high */
    PORTB.OUTSET = 0x03;

    /* Use standard 400 kHz timing */
    TWI0.CTRLA = TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc;
    TWI0.SADDR = I2C_ADDR << 1;
    /* Only match one address */
    TWI0.SADDRMASK = 0x00;
    TWI0.MCTRLA = 0;
    /* Auto-clear interrupt flag */
    TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_SMEN_bm | TWI_ENABLE_bm;
}