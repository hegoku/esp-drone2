#ifndef BUS_BUS_SPI_H
#define BUS_BUS_SPI_H

#include "bus/bus.h"

void init_spi(struct bus *bus);
int spi_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);
int spi_write(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len);
#endif