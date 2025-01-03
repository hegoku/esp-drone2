#ifndef BUS_UART_H
#define BUS_UART_H
#include "bus/bus.h"

void init_uart(struct bus *bus);
int uart_write(struct bus_dev *dev, unsigned char *data, int len);
#endif