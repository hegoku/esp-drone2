#ifndef DRIVERS_BUS_SPI_H
#define DRIVERS_BUS_SPI_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "bus/bus.h"

struct spi_dev_priv {
	spi_device_handle_t handle;
	spi_device_interface_config_t devcfg;
	unsigned char reg_addr;
};

struct spi_priv {
	gpio_num_t miso;
	gpio_num_t mosi;
	gpio_num_t clk;
	int speed;
	struct bus_dev dev_list[3];
	char dev_count;
};

#define SPI_GET_PRIV(x) ((struct spi_priv*)x)
#define SPI_DEV_GET_PRIV(x) ((struct spi_dev_priv*)x)

void init_spi(struct bus *bus);
int spi_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);
int spi_write(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);
#endif