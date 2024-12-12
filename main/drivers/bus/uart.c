#include "drivers/bus/uart.h"

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
	uart_set_pin(priv->port, priv->tx, priv->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}