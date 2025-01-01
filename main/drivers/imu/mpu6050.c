#include <string.h>
#include "drivers/imu/mpu6050.h"
#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "bus/i2c.h"

int mpu6050_write_reg_byte(struct bus_dev *dev, unsigned char reg, unsigned char byte)
{
	unsigned char buf = byte;
	return i2c_write(dev, reg, &buf, 1);
}

int mpu6050_read_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return i2c_read(dev, reg, buf, bytes);
}

int mpu6050_write_reg(struct bus_dev *dev, unsigned char reg, unsigned char *buf, int bytes)
{
	return i2c_write(dev, reg, buf, bytes);
}

unsigned char mpu6050_who_am_i(struct bus_dev *dev)
{
	unsigned char buf;
	mpu6050_read_reg(dev, MPU6050_REG_WHO_AM_I, &buf, 1);
	return buf;
}

int mpu6050_sensor_read(struct imu_sensor *sensor)
{
	unsigned char buf[14];
	mpu6050_read_reg(sensor->dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
	sensor->accel.raw.x = (((unsigned short)buf[0] << 8) | buf[1]);
	sensor->accel.raw.y = (((unsigned short)buf[2] << 8) | buf[3]);
	sensor->accel.raw.z = (((unsigned short)buf[4] << 8) | buf[5]);

	sensor->gyro.raw.x = (((unsigned short)buf[8] << 8) | buf[9]);
	sensor->gyro.raw.x = (((unsigned short)buf[10] << 8) | buf[11]);
	sensor->gyro.raw.x = (((unsigned short)buf[12] << 8) | buf[13]);

	sensor->temperature.raw = (((unsigned short)buf[6] << 8) | buf[7]);
	return 0;
}

int mpu6050_sensor_convert_data(struct imu_sensor *sensor)
{
	sensor->accel.value.x = sensor->accel.filtered.x / MPU6050_ACCEL_RESOLUTION;
	sensor->accel.value.y = sensor->accel.filtered.y / MPU6050_ACCEL_RESOLUTION;
	sensor->accel.value.z = sensor->accel.filtered.z / MPU6050_ACCEL_RESOLUTION;

	sensor->gyro.value.x = sensor->gyro.filtered.x / MPU6050_GYRO_RESOLUTION;
	sensor->gyro.value.y = sensor->gyro.filtered.y / MPU6050_GYRO_RESOLUTION;
	sensor->gyro.value.z = sensor->gyro.filtered.z / MPU6050_GYRO_RESOLUTION;

	if (strcmp(sensor->name, "MPU6050")==0) {
		sensor->temperature.value = 36.53 + (float)sensor->temperature.raw / 340.0;
	} else {
		sensor->temperature.value = 21.0 + (float)sensor->temperature.raw / 333.87;
	}
	return 0;
}

int mpu6050_prob(struct bus_dev *dev)
{
	unsigned char id = 0;
	for (int i = MPU6050_I2C_ADDRESS1; i <= MPU6050_I2C_ADDRESS2; i++)
	{
		dev->address = i;
		id = mpu6050_who_am_i(dev);
		if (id != MPU6050_WHOAMI_VALUE && id != MPU6500_WHOAMI_VALUE)
			continue;

		mpu6050_write_reg_byte(dev, MPU6050_REG_PWR_MGMT_1, 0x00);
		// mpu6050_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x07); //陀螺采样, 200Hz 125HZ
		// mpu6050_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x01); //陀螺采样, 500HZ
		mpu6050_write_reg_byte(dev, MPU6050_REG_SMPLRT_DIV, 0x00);	// 陀螺采样, 1000HZ MPU6050_SAMPLE_RATE
		mpu6050_write_reg_byte(dev, MPU6050_REG_CONFIG, 0x02);		// 低通滤波
		mpu6050_write_reg_byte(dev, MPU6050_REG_ACCEL_CONFIG, 0x00); // 加速度传感器 2g
		mpu6050_write_reg_byte(dev, MPU6050_REG_GYRO_CONFIG, 0x18);	// 陀螺椅传感器 2000deg/s
		mpu6050_write_reg_byte(dev, MPU6050_REG_PWR_MGMT_2, 0x00);	// xyz不进入待机
		mpu6050_write_reg_byte(dev, MPU6050_REG_INT_PIN_CFG, 0x92);	// 低电平触发
		if (id==MPU6050_WHOAMI_VALUE) {
			flight.imu.name = "MPU6050";
			dev->name = "MPU6050";
		} else {
			flight.imu.name = "MPU6500";
			dev->name = "MPU6500";
		}
		flight.imu.dev = dev;
		flight.imu.read = mpu6050_sensor_read;

		flight.imu.convert_data = mpu6050_sensor_convert_data;
		flight.imu.status = IMU_STATUS_ON;
		mpu6050_write_reg_byte(dev, MPU6050_REG_INT_ENABLE, 0x01);	// 开中断
		return 0;
	}

	return -1;
}