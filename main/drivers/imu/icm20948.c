#include "icm20948.h"
#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "bus/spi.h"

int icm20948_write_reg_byte(struct bus_dev *dev, unsigned char reg, unsigned char byte)
{
	unsigned char buf = byte;
	return spi_write(dev, reg, &buf, 1);
}

int icm20948_read_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return spi_read(dev, ICM20948_READ_REG(reg), buf, bytes);
}

int icm20948_write_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return spi_write(dev, reg, buf, bytes);
}

int icm20948_set_bank(struct bus_dev *dev, unsigned char bank)
{
	return icm20948_write_reg_byte(dev, ICM20948_REG_REG_BANK_SEL, bank);
}

unsigned char icm20948_who_am_i(struct bus_dev *dev)
{
	unsigned char buf;
	icm20948_set_bank(dev, ICM20948_BANK_0);
	icm20948_read_reg(dev, ICM20948_REG_WHO_AM_I, &buf, 1);
	return buf;
}

int icm20948_sensor_read(struct imu_sensor *sensor)
{
	unsigned char buf[14];
	icm20948_read_reg(sensor->dev, ICM20948_REG_ACCEL_XOUT_H, buf, 14);
	sensor->accel.raw.x = (((unsigned short)buf[0] << 8) | buf[1]);
	sensor->accel.raw.y = (((unsigned short)buf[2] << 8) | buf[3]);
	sensor->accel.raw.z = (((unsigned short)buf[4] << 8) | buf[5]);

	sensor->gyro.raw.x = (((unsigned short)buf[6] << 8) | buf[7]);
	sensor->gyro.raw.x = (((unsigned short)buf[8] << 8) | buf[9]);
	sensor->gyro.raw.x = (((unsigned short)buf[10] << 8) | buf[11]);

	sensor->temperature.raw = (((unsigned short)buf[12] << 8) | buf[13]);

	sensor->accel.unfiltered.x = ((float)sensor->accel.raw.x) / ICM20948_GYRO_RESOLUTION;
	sensor->accel.unfiltered.y = ((float)sensor->accel.raw.y) / ICM20948_GYRO_RESOLUTION;
	sensor->accel.unfiltered.z = ((float)sensor->accel.raw.z) / ICM20948_GYRO_RESOLUTION;

	sensor->gyro.unfiltered.x = ((float)sensor->gyro.raw.x) / ICM20948_GYRO_RESOLUTION;
	sensor->gyro.unfiltered.y = ((float)sensor->gyro.raw.y) / ICM20948_GYRO_RESOLUTION;
	sensor->gyro.unfiltered.z = ((float)sensor->gyro.raw.z) / ICM20948_GYRO_RESOLUTION;

	sensor->temperature.value = 21.0 + (float)sensor->temperature.raw / 333.87;
	return 0;
}

int icm20948_prob(struct bus_dev *dev)
{
	if (icm20948_who_am_i(dev)!=ICM20948_WHOAMI_VALUE)
		return -1;

	dev->name = "ICM20948";

	icm20948_write_reg_byte(dev, ICM20948_REG_PWR_MGMT_1, 0x80); //reset
	delay_ms(50);
	icm20948_write_reg_byte(dev, ICM20948_REG_PWR_MGMT_1, 0x1); //clock source
	icm20948_write_reg_byte(dev, ICM20948_REG_USER_CTRL, 0x30);	 // I2C master mode and set I2C_IF_DIS
	icm20948_write_reg_byte(dev, ICM20948_REG_INT_PIN_CFG, 0xC0); //INT1 pin is active low
	
	icm20948_set_bank(dev, ICM20948_BANK_2);
	icm20948_write_reg_byte(dev, ICM20948_REG_ODR_ALIGN_EN, 0x01);
	icm20948_write_reg_byte(dev, ICM20948_REG_GYRO_SMPLRT_DIV, 0x00); //sample rate 1125HZ
	icm20948_write_reg_byte(dev, ICM20948_REG_GYRO_CONFIG_1, 0x37); //enable DLPF,  ±2000 dps
	icm20948_write_reg_byte(dev, ICM20948_REG_GYRO_CONFIG_2, 0x1); //8x averaging
	icm20948_write_reg_byte(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0); //1125HZ
	icm20948_write_reg_byte(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0); //1125HZ
	icm20948_write_reg_byte(dev, ICM20948_REG_ACCEL_CONFIG, 0x31); //enable DLPF,  ±2g
	// icm20948_write_reg_byte(ICM20948_REG_ACCEL_CONFIG_2, 0x1); // 8x averaging
	icm20948_write_reg_byte(dev, ICM20948_REG_TEMP_CONFIG, 0x3); //1125HZ

	icm20948_set_bank(dev, ICM20948_BANK_0);
	icm20948_write_reg_byte(dev, ICM20948_REG_INT_ENABLE_1, 1);

	flight.imu.name = "ICM20948";
	flight.imu.dev = dev;
	flight.imu.read = icm20948_sensor_read;
	flight.imu.status = IMU_STATUS_ON;

	return 0;
}