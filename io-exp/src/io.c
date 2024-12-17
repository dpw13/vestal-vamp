#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include "io.h"
#include "spi.h"

uint8_t quad_state[2];
uint16_t quad_pos[2];
uint8_t quad_skip[2];
uint8_t io_update;

void io_print_state(void) {
    printf("IO update: %d Q0: %04x Q1: %04x Miss Q0: %04x Q1: %04x\n",
        io_update, quad_pos[0], quad_pos[1],
        quad_skip[0], quad_skip[1]);
}

static inline void update_quad(uint8_t idx, uint8_t pin_state) {
    quad_state[idx] = (pin_state | quad_state[idx] << 2) & 0x0F;
    switch (quad_state[idx]) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            /* Right */
            quad_pos[idx] += 1;
            io_update = 1;
            break;
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            /* Left */
            quad_pos[idx] -= 1;
            io_update = 1;
            break;
        case 0b0011:
        case 0b0110:
        case 0b1100:
        case 0b1001:
            /* Missed an update */
            quad_skip[idx] += 1;
            io_update = 1;
            break;
        /* Others: no change */
    }
}

static inline uint8_t read_encA(void) {
    uint8_t pa = VPORTA.IN & (_BV(5) | _BV(0));
    return (pa | (pa >> 4)) & 0x3;
}

ISR(PORTA_PORT_vect) {
    uint8_t q = read_encA();

    if ((VPORTA.IN & SPI_SS_PIN) == 0) {
        /* SPI callback on falling edge of ~SS */
        spi_start();
    }

    PORTA.INTFLAGS = PORTA.INTFLAGS;

    update_quad(0, q);
}

static inline uint8_t read_encB(void) {
    uint8_t pa = VPORTC.IN & (_BV(3) | _BV(2));
    return (pa >> 2);
}

ISR(PORTC_PORT_vect) {
    uint8_t q = read_encB();

    PORTC.INTFLAGS = PORTC.INTFLAGS;

    update_quad(1, q);
}

void io_init(void) {
    PORTA.DIR = PORTA_DIR_VAL;
    PORTB.DIR = PORTB_DIR_VAL;
    PORTC.DIR = PORTC_DIR_VAL;

    /* Input configs */
    PORTA.PIN0CTRL = DI_PINCTRL; // EncA.X
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // ~SS
    PORTA.PIN5CTRL = DI_PINCTRL; // EncA.Y
    PORTA.PIN6CTRL = AI_PINCTRL;
    PORTA.PIN7CTRL = AI_PINCTRL;

    PORTB.PIN0CTRL = AI_PINCTRL;
    PORTB.PIN1CTRL = AI_PINCTRL;
    PORTB.PIN4CTRL = AI_PINCTRL;
    PORTB.PIN5CTRL = AI_PINCTRL;

    PORTC.PIN2CTRL = DI_PINCTRL; // EncB.X
    PORTC.PIN3CTRL = DI_PINCTRL; // EncB.Y

    quad_pos[0] = 0;
    quad_pos[1] = 0;
    quad_skip[0] = 0;
    quad_skip[1] = 0;

    quad_state[0] = read_encA();
    quad_state[1] = read_encB();

    io_update = 0;
}
