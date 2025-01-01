#ifndef DRIVERS_BUS_I2C_H
#define DRIVERS_BUS_I2C_H

#include <driver/gpio.h>
#include <driver/i2c.h>
#include "bus/i2c.h"

#define I2C_MASTER_ACK_EN 1

struct i2c_priv {
	gpio_num_t sda;
	gpio_num_t scl;
	i2c_port_t port;
	int speed;
	struct bus_dev *dev_list;
};

#define I2C_GET_PRIV(x) ((struct i2c_priv*)x)

#endif