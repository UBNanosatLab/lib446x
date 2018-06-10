#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

#ifndef HIGH
#define HIGH 0x1
#endif

#ifndef LOW
#define LOW  0x0
#endif

#ifndef INPUT
#define INPUT 0x0
#endif

#ifndef OUTPUT
#define OUTPUT 0x1
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern void spi_init();
extern void spi_write_byte(unsigned char b);
extern void spi_write_data(int len, const unsigned char *data);
extern void spi_read_data(int len, unsigned char *data);

extern void gpio_config(int pin, int mode);
extern void gpio_write(int pin, int value);

extern void delay_micros(unsigned long micros);

#ifdef __cplusplus
}
#endif

#endif
