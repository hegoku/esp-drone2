#include "bus_tree.h"

#define PIN_NUM_MISO GPIO_NUM_16
#define PIN_NUM_MOSI GPIO_NUM_18
#define PIN_NUM_CLK GPIO_NUM_19

static struct bus_driver bus_tree[] = {
	{
		.name="uart1",
		.type=1,
		.priv.uart={
			.port=UART_NUM_0,
			.baud_rate=115200,
			.tx=0,
			.rx=0
		}
	},
	{
		.name="uart2",
		.type=1,
		.priv.uart={
			.port=UART_NUM_1,
			.baud_rate=115200,
			.tx=0,
			.rx=0
		}
	},
	{
		.name="uart3",
		.type=1,
		.priv.uart={
			.port=UART_NUM_2
			.baud_rate=115200,
			.tx=0,
			.rx=0
		}
	},
	{
		.name="spi1",
		.type=2,
		.priv.spi={
			.miso=PIN_NUM_MISO,
			.mosi=PIN_NUM_MOSI,
			.clk=PIN_NUM_CLK,
			.speed=7 * 1000 * 1000
		}
	}
};

void init_bus()
{
	int size = sizeof(bus_tree) / sizeof(bus_tree[0]);
	for (int i = 0; i < size;i++) {
		switch (bus_tree[i].type)
		{
		case 1:
			init_uart(bus_tree[i].priv.uart.port, bus_tree[i].priv.uart.tx, bus_tree[i].priv.uart.rx, bus_tree[i].priv.baud_rate);
			break;
		case 2:
			init_spi(bus_tree[i].priv.spi.miso, bus_tree[i].priv.spi.mosi, bus_tree[i].priv.spi.clk);
			break;
		default:
			break;
		}
	}
}