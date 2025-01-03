#ifndef DRIVERS_BUS_SPI_H
#define DRIVERS_BUS_SPI_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "bus/bus.h"
#include "bus/spi.h"

struct spi_dev_priv {
	spi_device_handle_t handle;
	spi_device_interface_config_t devcfg;
};

struct spi_priv {
	gpio_num_t miso;
	gpio_num_t mosi;
	gpio_num_t clk;
	spi_host_device_t host_id;
	int speed;
	struct bus_dev dev_list[3];
	char dev_count;
};

#define SPI_GET_PRIV(x) ((struct spi_priv*)x)
#define SPI_DEV_GET_PRIV(x) ((struct spi_dev_priv*)x)

#endif