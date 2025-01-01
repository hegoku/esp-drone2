#include <string.h>
#include "drivers/bus/spi.h"

void init_spi(struct bus *bus)
{
	struct spi_priv *priv = SPI_GET_PRIV(bus->priv);
	struct spi_dev_priv *dev_priv;
	int (*spi_dev_probs)(struct bus_dev *dev);

	esp_err_t ret;
	spi_bus_config_t buscfg={
        .miso_io_num=priv->miso,
        .mosi_io_num=priv->mosi,
        .sclk_io_num=priv->clk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=32
    };
	ret=spi_bus_initialize(priv->host_id, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
	for (int i=0;i<priv->dev_count;i++) {
		priv->dev_list[i].bus = bus;
		priv->dev_list[i].priv = malloc(sizeof(struct spi_dev_priv));
		dev_priv = SPI_DEV_GET_PRIV(priv->dev_list[i].priv);
		memset(dev_priv, 0, sizeof(struct spi_dev_priv));
		dev_priv->devcfg.clock_speed_hz = priv->speed;
		dev_priv->devcfg.mode = 3;							// SPI mode 3
		dev_priv->devcfg.spics_io_num = (gpio_num_t)priv->dev_list[i].address;				// CS pin
		dev_priv->devcfg.queue_size = 7;					// We want to be able to queue 7 transactions at a time
		dev_priv->devcfg.address_bits = 8;
		ESP_ERROR_CHECK(spi_bus_add_device(priv->host_id, &dev_priv->devcfg, &dev_priv->handle));
		for (int j=0;j<BUS_MAX_PROBS_NR;j++) {
			if (!(*bus->probs)[j]) continue;
			spi_dev_probs = (*bus->probs)[j];
			if (spi_dev_probs(&priv->dev_list[i])==0) {
				bus_add_dev(bus, &priv->dev_list[i]);
				break;
			}
		}
	}
}

int spi_write(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len)
{
	int ret;
	static struct spi_transaction_t trans;
	struct spi_dev_priv *dev_priv = SPI_DEV_GET_PRIV(dev->priv);

	trans.tx_buffer = data;
	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = 8*len;

	ret = spi_device_transmit(dev_priv->handle, &trans);

	return ret;
}

int spi_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len) {
	int ret;
	static struct spi_transaction_t trans;
	struct spi_dev_priv *dev_priv = SPI_DEV_GET_PRIV(dev->priv);

	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = len * 8;
	trans.rxlength = len * 8;
	trans.rx_buffer = data;

	ret = 0;
	ret = spi_device_transmit(dev_priv->handle, &trans);

	return ret;
}