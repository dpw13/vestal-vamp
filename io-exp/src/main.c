#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "adc.h"
#include "spi.h"
#include "io.h"
#include "uart.h"

#define LED_PORT    PORTC
#define LED         _BV(0)

ISR(BADISR_vect) {
    /* Default interrupt vector */
    LED_PORT.OUTTGL = LED;
    puts("Unexpected IRQ");
    printf("CTRLA = %02x\n", CPUINT.CTRLA);
    printf("STATUS = %02x\n", CPUINT.STATUS);
    while(1);
}

void print_state(void) {
    putchar('\n');
    io_print_state();
    adc_print_state();
    spi_print_state();
    printf("CPUINT.STATUS = %02x\n", CPUINT.STATUS);
}

int main (void)
{
    /* Turn off LED */
    LED_PORT.OUTSET = LED;

    /* Configure clock */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);

    /* Select interrupt vector at 0x200 */
    _PROTECTED_WRITE(CPUINT.CTRLA, 0);
    io_init();
    uart_init();
    adc_init();
    spi_init();
    sei();

    puts("IO expander ready");

    while(1) {
        int input;

        if (io_update) {
            io_update = 0;
            io_print_state();
        }
        if (adc_update) {
            adc_update = 0;
            adc_print_state();
        }

        input = uart_getchar_nonblock();
        if (input < 0) {
            _delay_ms(2000);
            print_state();
        } else {
            uart_putchar(input, NULL);

            switch (input) {
                case 'r':
                    /* Reset to bootloader */
                    puts("\nResetting...\n");
                    uart_flush();
                    _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
                    break;
                case 'i':
                    print_state();
                    break;
                case 'l':
                    LED_PORT.OUTTGL = LED;
                    break;
            }
        }
    }
}
