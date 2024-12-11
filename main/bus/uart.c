#include "uart.h"

void init_uart(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int buad_rate)
{
	uart_config_t uart_config = {
		.baud_rate = buad_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
		.source_clk = UART_SCLK_DEFAULT
	};
	ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
	uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}