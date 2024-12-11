#ifndef BUS_UART_H
#define BUS_UART_H
#include <driver/gpio.h>
#include <driver/uart.h>
#include "bus.h"

struct uart_priv {
	uart_port_t port;
	gpio_num_t tx;
	gpio_num_t rx;
	int baud_rate;
};

void init_uart(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int buad_rate);
#endif