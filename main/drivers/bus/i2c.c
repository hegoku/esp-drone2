#include "drivers/bus/i2c.h"
#include <driver/i2c.h>
#include <string.h>

void init_i2c(struct bus *bus)
{
	int (*i2c_dev_probs)(struct bus_dev *dev);

	struct i2c_priv *priv = I2C_GET_PRIV(bus->priv);
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = priv->sda,         // select GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = priv->scl,         // select GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = priv->speed,  // select frequency specific to your project
	};
	ESP_ERROR_CHECK(i2c_param_config(priv->port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(priv->port, conf.mode, 0, 0, 0));
	for (int j=0;j<BUS_MAX_PROBS_NR;j++) {
		if (!(*bus->probs)[j]) continue;
		i2c_dev_probs = (*bus->probs)[j];
		struct bus_dev *dev = malloc(sizeof(struct bus_dev));
		memset(dev, 0, sizeof(struct bus_dev));
		dev->bus = bus;
		if (i2c_dev_probs(dev) == 0)
		{
			bus_add_dev(bus, dev);
		}
		else
		{
			free(dev);
		}
	}
}

int i2c_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev->address | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
	i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev->address | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
	i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	esp_err_t err = i2c_master_cmd_begin(0, cmd, (TickType_t)5);
	i2c_cmd_link_delete(cmd);
	return err;
}

int i2c_write(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev->address  | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
	i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK_EN);
    i2c_master_write(cmd, (unsigned char *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(0, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);
	return err;
}