#include <driver/spi_master.h>
#include "spi.h"

struct spi_dev_map {
	spi_device_handle_t handle;
	spi_device_interface_config_t devcfg;
	struct bus_device *dev;
};

struct spi_dev_map dev_map[3] = {0};
int dev_map_count = 0;

void init_spi(gpio_num_t miso, gpio_num_t mosi, gpio_num_t clk)
{
	esp_err_t ret;
	spi_bus_config_t buscfg={
        .miso_io_num=miso,
        .mosi_io_num=mosi,
        .sclk_io_num=clk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=32
    };
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
}

void spi_register_dev(struct bus_device *dev)
{
	if (dev_map_count>=3) {
		return;
	}
	dev_map[dev_map_count].dev = dev;
	dev_map[dev_map_count].devcfg.clock_speed_hz = 7 * 1000 * 1000; //Clock out at 7 MHz
	dev_map[dev_map_count].devcfg.mode = 3;							// SPI mode 3
	dev_map[dev_map_count].devcfg.spics_io_num = (gpio_num_t)dev->address;				// CS pin
	dev_map[dev_map_count].devcfg.queue_size = 7;					// We want to be able to queue 7 transactions at a time
	dev_map[dev_map_count].devcfg.address_bits = 8;
	spi_bus_add_device(HSPI_HOST, &dev_map[dev_map_count].devcfg, &dev_map[dev_map_count].handle);
	dev_map_count++;
}

int spi_write(struct bus_device *dev, unsigned char reg_addr, unsigned char *data, int len)
{
	int ret;
	static struct spi_transaction_t trans;
	struct spi_dev_map *finded_dev=0;
	for (int i = 0; i < dev_map_count;i++) {
		if (dev_map[i].dev==dev) {
			finded_dev = &dev_map[i];
		}
	}
	if (finded_dev==0) {
		return 0;
	}

	trans.tx_buffer = data;
	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = 8*len;

	ret = spi_device_transmit(finded_dev->handle, &trans);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

int spi_read(struct bus_device *dev, unsigned char reg_addr, unsigned char *data, int len) {
	int ret;
	static struct spi_transaction_t trans;
	struct spi_dev_map *finded_dev=0;
	for (int i = 0; i < dev_map_count;i++) {
		if (dev_map[i].dev==dev) {
			finded_dev = &dev_map[i];
		}
	}
	if (finded_dev==0) {
		return 0;
	}

	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = len * 8;
	trans.rxlength = len * 8;
	trans.rx_buffer = data;

	ret = 0;
	ret = spi_device_transmit(finded_dev->handle, &trans);
	if (ret < 0) {
		return ret;
	}

	return ret;
}