#include <driver/gpio.h>
#include <stdio.h>
#include "platform/bus/spi.h"
#include "platform/bus/uart.h"
#include "platform/bus/i2c.h"
#include "platform/gpio_config.h"

#include "drivers/anotc_uart.h"

extern int (*spi_dev_probs[])(struct bus_dev *dev);
extern int (*uart_dev_probs[])(struct bus_dev *dev);
extern int (*i2c_dev_probs[])(struct bus_dev *dev);

struct spi_priv spi1_config = {
	.miso=SPI1_MISO_GPIO,
	.mosi=SPI1_MOSI_GPIO,
	.clk=SPI1_CLK_GPIO,
	.host_id=VSPI_HOST,
	.speed=1000*1000,
	.dev_count=3,
	.dev_list={
		{
			.address=SPI1_CS1_GPIO
		},
		{
			.address=SPI1_CS2_GPIO
		},
		{
			.address=SPI1_CS3_GPIO
		}
	}
};

struct uart_priv uart1_config = {
	.baud_rate = 460800,
	.tx_pin=UART1_TX_GPIO,
	.rx_pin=UART1_RX_GPIO,
	.port=UART_NUM_0,
	.rx_data_size=64,
	.rx_buffer_size=256,
	.tx_buffer_size=1024,
	.dev_init=anotc_uart_init
};

struct i2c_priv i2c1_config = {
	.speed = 400000,
	.sda = I2C1_SDA_GPIO,
	.scl = I2C1_SCL_GPIO,
	.port = I2C_NUM_0,
	.dev_list = NULL
};

struct bus bus_tree[] = {
	// {
	// 	.name="UART1",
	// 	.priv=&uart1_config,
	// 	.init=init_uart
	// },
	{
		.name="SPI1",
		.priv=&spi1_config,
		.probs=&spi_dev_probs,
		.init=init_spi
	},
	{
		.name="I2C1",
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