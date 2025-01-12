#include <driver/gpio.h>
#include <stdio.h>
#include "drivers/bus/spi.h"
#include "drivers/bus/uart.h"
#include "drivers/bus/i2c.h"

#include "drivers/anotc_uart.h"

extern int (*spi_dev_probs[])(struct bus_dev *dev);
extern int (*uart_dev_probs[])(struct bus_dev *dev);
extern int (*i2c_dev_probs[])(struct bus_dev *dev);

struct spi_priv spi1_config = {
	.miso=GPIO_NUM_16,
	.mosi=GPIO_NUM_18,
	.clk=GPIO_NUM_19,
	.host_id=HSPI_HOST,
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
	.tx_pin=GPIO_NUM_1,
	.rx_pin=GPIO_NUM_3,
	.port=UART_NUM_0,
	.rx_buffer_size=1024,
	.tx_buffer_size=1024,
	.dev_init=anotc_uart_init
};

struct i2c_priv i2c1_config = {
	.speed = 400000,
	.sda = GPIO_NUM_27,
	.scl = GPIO_NUM_14,
	.port = I2C_NUM_0,
	.dev_list = NULL
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
	},
	{
		.name="i2c1",
		.priv=&i2c1_config,
		.probs=&i2c_dev_probs,
		.init=init_i2c
	}
};

void init_bus_tree()
{
	bus_init(bus_tree, sizeof(bus_tree)/sizeof(struct bus));
}

void print_bus_tree()
{
	struct bus_dev *dev;
	for (int i = 0; i < sizeof(bus_tree) / sizeof(struct bus); i++)
	{
		printf("%s\n", bus_tree[i].name);
		dev = bus_tree[i].dev_list;
		while (dev) {
			printf("  |- %s(0x%x)\n", dev->name, dev->address);
			dev = dev->next;
		}
	}
}