#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "drivers/bus/uart.h"

void uart_read_task(void *param)
{
	struct bus_dev *dev = (struct bus_dev*) param;
	struct uart_priv *priv = UART_GET_PRIV(dev->bus->priv);
	unsigned char* data = (unsigned char*) malloc(128);
    while (1) {
        const int len = uart_read_bytes(UART_NUM_1, data, 128, 10);
        if (len > 0) {
			priv->read_handler(data, len);
		}
	}
    free(data);
}

void init_uart(struct bus *bus)
{
	struct uart_priv *priv = UART_GET_PRIV(bus->priv);

	uart_config_t uart_config = {
		.baud_rate = priv->baud_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
		.source_clk = UART_SCLK_DEFAULT
	};
	ESP_ERROR_CHECK(uart_param_config(priv->port, &uart_config));
	uart_set_pin(priv->port, priv->tx_pin, priv->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	ESP_ERROR_CHECK(uart_driver_install(priv->port, priv->rx_buffer_size, priv->tx_buffer_size, 0, NULL, 0));

	struct bus_dev *dev = (struct bus_dev*)malloc(sizeof(struct bus_dev));
	memset(dev, 0, sizeof(struct bus_dev));
	dev->bus = bus;
	priv->dev_init(dev);
	bus_add_dev(bus, dev);

	xTaskCreate(uart_read_task, dev->name, 1024 * 2, (void*)dev, 5, NULL);
}

int uart_write(struct bus_dev *dev, unsigned char *data, int len)
{
	struct uart_priv *priv = UART_GET_PRIV(dev->bus->priv);
	return uart_write_bytes(priv->port, (const void*)data, len);
}