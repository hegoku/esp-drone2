#ifndef BUS_SPI_H
#define BUS_SPI_H

#include <driver/gpio.h>
#include "bus.h"

struct spi_priv {
	gpio_num_t miso;
	gpio_num_t mosi;
	gpio_num_t clk;
	int speed;
};

void init_spi(gpio_num_t miso, gpio_num_t mosi, gpio_num_t clk);
void spi_register_dev(struct bus_device *dev);
int spi_write(struct bus_device *dev, unsigned char reg_addr, unsigned char *data, int len);
int spi_read(struct bus_device *dev, unsigned char reg_addr, unsigned char *data, int len);

#endif