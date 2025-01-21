#include <string.h>
#include "drivers/imu/mpu6050.h"
#include "drivers/imu/mpu6500.h"
#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "bus/spi.h"
#include <stdio.h>

int mpu6500_write_reg_byte(struct bus_dev *dev, unsigned char reg, unsigned char byte)
{
	unsigned char buf = byte;
	return spi_write(dev, reg, &buf, 1);
}

int mpu6500_read_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return spi_read(dev, reg, buf, bytes);
}

int mpu6500_write_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return spi_write(dev, reg, buf, bytes);
}

unsigned char mpu6500_who_am_i(struct bus_dev *dev)
{
	unsigned char buf;
	mpu6500_read_reg(dev, MPU6050_REG_WHO_AM_I, &buf, 1);
	return buf;
}

int mpu6500_sensor_read(struct imu_sensor *sensor)
{
	unsigned char buf[14];
	mpu6500_read_reg(sensor->dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
	sensor->accel.raw.x = (((short)buf[0] << 8) | buf[1]);
	sensor->accel.raw.y = (((short)buf[2] << 8) | buf[3]);
	sensor->accel.raw.z = (((short)buf[4] << 8) | buf[5]);

	sensor->gyro.raw.x = (((short)buf[8] << 8) | buf[9]);
	sensor->gyro.raw.y = (((short)buf[10] << 8) | buf[11]);
	sensor->gyro.raw.z = (((short)buf[12] << 8) | buf[13]);

	sensor->temperature.raw = (((short)buf[6] << 8) | buf[7]);

	sensor->accel.unfiltered.x = ((float)sensor->accel.raw.x) / MPU6050_ACCEL_RESOLUTION;
	sensor->accel.unfiltered.y = ((float)sensor->accel.raw.y) / MPU6050_ACCEL_RESOLUTION;
	sensor->accel.unfiltered.z = ((float)sensor->accel.raw.z) / MPU6050_ACCEL_RESOLUTION;

	sensor->gyro.unfiltered.x = ((float)sensor->gyro.raw.x) / MPU6050_GYRO_RESOLUTION;
	sensor->gyro.unfiltered.y = ((float)sensor->gyro.raw.y) / MPU6050_GYRO_RESOLUTION;
	sensor->gyro.unfiltered.z = ((float)sensor->gyro.raw.z) / MPU6050_GYRO_RESOLUTION;

	if (strcmp(sensor->name, "MPU6050")==0) {
		sensor->temperature.value = 36.53 + ((float)sensor->temperature.raw) / 340.0;
	} else {
		sensor->temperature.value = 21.0 + ((float)sensor->temperature.raw) / 333.87;
	}
	return 0;
}

void mpu6500_init(struct bus_dev *dev, unsigned int id)
{
	mpu6500_write_reg_byte(dev, MPU6050_REG_PWR_MGMT_1, 0x00);
	// mpu6500_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x07); //陀螺采样, 200Hz 125HZ
	// mpu6500_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x01); //陀螺采样, 500HZ
	mpu6500_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x00);	// 陀螺采样, 1000HZ MPU6050_SAMPLE_RATE
	mpu6500_write_reg_byte(dev, MPU6050_REG_CONFIG, 0x02);		// 低通滤波
	mpu6500_write_reg_byte(dev, MPU6050_REG_ACCEL_CONFIG, 0x00); // 加速度传感器 2g
	mpu6500_write_reg_byte(dev, MPU6050_REG_GYRO_CONFIG, 0x18);	// 陀螺椅传感器 2000deg/s
	mpu6500_write_reg_byte(dev, MPU6050_REG_PWR_MGMT_2, 0x00);	// xyz不进入待机
	mpu6500_write_reg_byte(dev, MPU6050_REG_INT_PIN_CFG, 0x92);	// 低电平触发
	if (id==MPU6050_WHOAMI_VALUE) {
		flight.imu.name = "MPU6050";
		dev->name = "MPU6050";
	} else {
		flight.imu.name = "MPU6500";
		dev->name = "MPU6500";
	}
	flight.imu.dev = dev;
	flight.imu.read = mpu6500_sensor_read;

	flight.imu.status = IMU_STATUS_ON;
	flight.imu.freq = 1000;
	mpu6500_write_reg_byte(dev, MPU6050_REG_INT_ENABLE, 0x01);	// 开中断
}

int mpu6500_spi_prob(struct bus_dev *dev)
{
	unsigned char id = mpu6500_who_am_i(dev);
	printf("6500:%d\n", id);
	if (id != MPU6050_WHOAMI_VALUE && id != MPU6500_WHOAMI_VALUE) {
		return -1;
	}

	mpu6500_init(dev, id);
	return 0;
}