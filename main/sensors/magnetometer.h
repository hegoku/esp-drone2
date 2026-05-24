#ifndef SENSORS_MAGNETOMETER_H
#define SENSORS_MAGNETOMETER_H

#include "bus/bus.h"
#include "sensors/sensor_rotation.h"

#define MAGNETOMETER_STATUS_ON 0x1
#define MAGNETOMETER_STATUS_DTRY 0x02

struct magnetometer_sensor {
	char *name;
	enum sensor_rotation rotation;
	unsigned char status;
	struct {
		short x;
		short y;
		short z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} value;
	struct {
		short raw;
		float value;
	} temperature;
	struct {
		float x_k[3];
		float x_offset;
		float y_k[3];
		float y_offset;
		float z_k[3];
		float z_offset;
	} calibration;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct magnetometer_sensor *sensor);
	unsigned short freq;

	float declination;
};

#define IS_MAGNETOMETER_ON(sensor) ((sensor).status & MAGNETOMETER_STATUS_ON)
#define IS_MAGNETOMETER_DTRY(sensor) ((sensor).status & MAGNETOMETER_STATUS_DTRY)

void init_magnetometer(struct magnetometer_sensor *sensor);
void magnetometer_filter(struct magnetometer_sensor *sensor);
#endif