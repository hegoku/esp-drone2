#ifndef PLATFORM_BUS_UART_H
#define PLATFORM_BUS_UART_H
#include <driver/gpio.h>
#include <driver/uart.h>
#include "bus/bus.h"
#include "bus/uart.h"

struct uart_priv {
	uart_port_t port;
	gpio_num_t tx_pin;
	gpio_num_t rx_pin;
	int baud_rate;
	int rx_data_size;
	int rx_buffer_size;
	int tx_buffer_size;

	int (*dev_init)(struct bus_dev *dev);

	void (*read_handler)(unsigned char *data, int len);
};

#define UART_GET_PRIV(x) ((struct uart_priv*)x)

#endif