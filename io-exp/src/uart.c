#include <avr/io.h>
#include <stdio.h>
#include <errno.h>
#include "uart.h"

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

#define BAUD_SETTING (((20000000/6)*64) / (16L*BAUD_RATE))
#define BAUD_ACTUAL ((64L*(20000000/6)) / (16L*BAUD_SETTING))

void uart_init(void) {
    USART0.BAUD = BAUD_SETTING;

    USART0.CTRLA = USART_RS485_OFF_gc;
    USART0.CTRLB = USART_RXMODE_NORMAL_gc;

    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
    USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;   /* Enable RX and TX */

    /* Set TX pin high (as per TRM) */
    PORTB.OUTSET = _BV(2);

    stdout = &uart_output;
    stdin  = &uart_input;
}

int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(USART0.STATUS, USART_DREIF_bp);
    USART0.TXDATAL = c;

    return 0;
}

int uart_getchar(FILE *stream) {
    loop_until_bit_is_set(USART0.STATUS, USART_RXCIF_bp); /* Wait until data exists. */
    return USART0.RXDATAL;
}

int uart_getchar_nonblock() {
    if ((USART0.STATUS & USART_RXCIF_bm) == 0x00) {
        return -EAGAIN;
    } else {
        return USART0.RXDATAL;
    }
}

void uart_flush(void) {
    /* Clear TXCIF */
    USART0.STATUS = USART_TXCIF_bm;
    loop_until_bit_is_set(USART0.STATUS, USART_TXCIF_bp);
}