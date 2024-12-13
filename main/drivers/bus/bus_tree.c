#include <driver/gpio.h>
#include "drivers/bus/spi.h"
#include "drivers/bus/uart.h"

extern int (*spi_dev_probs[])(struct bus_dev *dev);
extern int (*uart_dev_probs[])(struct bus_dev *dev);

struct spi_priv spi1_config = {
	.miso=GPIO_NUM_16,
	.mosi=GPIO_NUM_18,
	.clk=GPIO_NUM_19,
	.speed=7*1000*1000,
	.dev_count=1,
	.dev_list={
		{
			.address=GPIO_NUM_5
		}
	}
};

struct uart_priv uart1_config = {
	.baud_rate = 115200,
	.tx=0,
	.rx=0,
	.port=0
};


struct bus bus_tree[] = {
	{
		.name="uart1",
		.priv=&uart1_config,
		.init=init_uart
	},
	{
		.name="spi1",
		.priv=&spi1_config,
		.probs=&spi_dev_probs,
		.init=init_spi
	}
};

void init_bus_tree()
{
	bus_init(bus_tree, sizeof(bus_tree)/sizeof(struct bus));
}