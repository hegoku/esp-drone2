#include "drivers/compass/lis3mdl.h"
#include "sensors/compass.h"
#include "bus/spi.h"
#include "flight/flight.h"

int lis3mdl_read_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return spi_read(dev, reg, buf, bytes);
}

int lis3mdl_write_reg(struct bus_dev *dev, unsigned char reg, unsigned char value)
{
	unsigned char buf = value;
	return spi_write(dev, reg, &buf, 1);
}

unsigned char lis3mdl_who_am_i(struct bus_dev *dev)
{
	unsigned char buf;
	lis3mdl_read_reg(dev, LIS3MDL_REG_WHO_AM_I, &buf, 1);
	return buf;
}

int lis3mdl_sensor_read(struct compass_sensor *sensor)
{
	unsigned char buf[8];
	lis3mdl_read_reg(sensor->dev, LIS3MDL_REG_OUT_X_L, buf, 8);
	sensor->raw.x = (((short)buf[1] << 8) | buf[0]);
	sensor->raw.y = (((short)buf[3] << 8) | buf[2]);
	sensor->raw.z = (((short)buf[5] << 8) | buf[4]);

	sensor->value.x = ((float)sensor->raw.x) / LIS3MDL_RESOLUTION_4;
	sensor->value.y = ((float)sensor->raw.y) / LIS3MDL_RESOLUTION_4;
	sensor->value.z = ((float)sensor->raw.z) / LIS3MDL_RESOLUTION_4;

	sensor->temperature.raw = (((short)buf[13] << 8) | buf[12]);
	sensor->temperature.value = ((float)sensor->temperature.raw) / 8.0;
	return 0;
}

int lis3mdl_prob(struct bus_dev *dev)
{
	if (lis3mdl_who_am_i(dev)!=LIS3MDL_WHOAMI_VALUE)
		return -1;

	dev->name = "LIS3MDL";

	lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG2, 0x4);
	lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG3, 0x0);
	lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG1, 0xF8);
	lis3mdl_write_reg(dev, LIS3MDL_REG_CTRL_REG4, 0xC);
	lis3mdl_write_reg(dev, LIS3MDL_REG_INT_CFG, 0);
	
	flight.compass.name = "LIS3MDL";
	flight.compass.dev = dev;
	flight.compass.read = lis3mdl_sensor_read;
	flight.compass.status = COMPASS_STATUS_ON;

	return 0;
}