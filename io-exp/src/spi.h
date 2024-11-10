#ifndef __SPI_H__
#define __SPI_H__

#define SPI_SS_PIN _BV(4)

void spi_init(void);
void spi_print_state(void);
void spi_start(void);

#endif /* __SPI_H__ */