#ifndef BUS_I2C_H
#define BUS_I2C_H

#include "bus/bus.h"

void init_i2c(struct bus *bus);
int i2c_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);
int i2c_write(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);

#endif