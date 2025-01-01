#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "bus/bus.h"

enum imu_status {
	IMU_STATUS_OFF,
	IMU_STATUS_ON
};

struct imu_data {
	struct {
		unsigned short x;
		unsigned short y;
		unsigned short z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} unfiltered;
	struct {
		float x;
		float y;
		float z;
	} value;
	struct {
		float x_k;
		float x_offset;
		float y_k;
		float y_offset;
		float z_k;
		float z_offset;
	} calibration;
};

struct imu_sensor {
	char *name;
	enum imu_status status;
	struct imu_data accel;
	struct imu_data gyro;
	struct {
		unsigned short raw;
		float value;
	} temperature;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct imu_sensor *sensor);
};

#endif