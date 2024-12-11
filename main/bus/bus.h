#ifndef BUS_BUS_H
#define BUS_BUS_H

#include "uart.h"
#include "spi.h"

struct bus_driver {
	char *name;
	char type;
	union
	{
		struct uart_priv uart;
		struct spi_priv spi;
	} priv;
};

struct bus_device
{
	char *name;
	unsigned int address;
	int (*init)(struct bus_device *dev);
};

#endif