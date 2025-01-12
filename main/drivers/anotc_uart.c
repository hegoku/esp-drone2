#include "bus/uart.h"
#include "drivers/bus/uart.h"
#include "anotc/anotc.h"
#include "drivers/anotc_uart.h"

static struct bus_dev *anotc_uart_dev;

void anotc_uart_send(unsigned char *data, int len)
{
	uart_write(anotc_uart_dev, data, len);
}

int anotc_uart_init(struct bus_dev *dev)
{
	struct uart_priv *priv = UART_GET_PRIV(dev->bus->priv);

	anotc_uart_dev = dev;
	dev->name = "ANOTC_UART";
	priv->read_handler = anotc_decode;

	anotc_set_send_func(anotc_uart_send);
	return 0;
}
