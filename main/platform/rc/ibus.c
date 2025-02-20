#include <driver/gpio.h>
#include "platform/bus/uart.h"
#include "rc/ibus.h"

static struct bus_dev *uart_dev;

int ibus_uart_init(struct bus_dev *dev)
{
	struct uart_priv *priv = UART_GET_PRIV(dev->bus->priv);

	uart_dev = dev;
	dev->name = "IBUS";
	priv->read_handler = ibus_decode;
	return 0;
}

static struct uart_priv uart_config = {
	.baud_rate = 115200,
	.tx_pin=GPIO_NUM_0,
	.rx_pin=GPIO_NUM_34,
	.port=UART_NUM_1,
	.rx_buffer_size=256,
	.tx_buffer_size=0,
	.dev_init=ibus_uart_init
};

struct bus uart = {
	.name="UART2",
	.priv = &uart_config,
	.init = init_uart
};

void init_ibus()
{
	uart.init(&uart);
}