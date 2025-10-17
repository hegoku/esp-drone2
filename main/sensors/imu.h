#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "bus/bus.h"
#include "sensors/sensor_rotation.h"
#include "misc/low_pass_filter_2p.h"

enum imu_status {
	IMU_STATUS_OFF,
	IMU_STATUS_ON
};

struct imu_data {
	struct {
		short x;
		short y;
		short z;
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
	enum sensor_rotation rotation;
	struct imu_data accel;
	struct imu_data gyro;
	struct {
		short raw;
		float value;
	} temperature;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct imu_sensor *sensor);
	unsigned short freq;

	unsigned char gyro_auto_calibration;
	unsigned short gyro_lpf_cutoff;
	unsigned short accel_lpf_cutoff;

	struct low_pass_filter_2p_param gyro_lpf[3];
	struct low_pass_filter_2p_param accel_lpf[3];
};

void init_imu(struct imu_sensor *sensor);
void imu_filter(struct imu_sensor *sensor);
void imu_calibration(struct imu_sensor *sensor);
#endif