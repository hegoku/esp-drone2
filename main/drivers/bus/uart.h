#ifndef DRIVERS_BUS_UART_H
#define DRIVERS_BUS_UART_H
#include <driver/gpio.h>
#include <driver/uart.h>
#include "bus/bus.h"

struct uart_priv {
	uart_port_t port;
	gpio_num_t tx;
	gpio_num_t rx;
	int baud_rate;
};

#define UART_GET_PRIV(x) ((struct uart_priv*)x)

void init_uart(struct bus *bus);
#endif