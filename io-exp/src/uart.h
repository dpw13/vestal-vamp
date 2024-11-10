#ifndef __SERIAL_H__
#define __SERIAL_H__

#define BAUD_RATE    115200

int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
int uart_getchar_nonblock(void);
void uart_flush(void);

void uart_init(void);

/* http://www.ermicro.com/blog/?p=325 */

extern FILE uart_output;
extern FILE uart_input;
extern FILE uart_io;

#endif /* __SERIAL_H__ */
