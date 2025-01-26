#ifndef SENSORS_BAROMETER_H
#define SENSORS_BAROMETER_H

#include "bus/bus.h"

#define BARO_STATUS_ON 0x1
#define BARO_STATUS_DTRY 0x2

struct barometer_sensor {
	char *name;
	unsigned char status;
	struct {
		unsigned int raw;
		float value;
	} pressure;
	struct {
		int raw;
		float value;
	} temperature;
	float altitude;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct barometer_sensor *sensor);
};

#define IS_BARO_ON(sensor) ((sensor).status & BARO_STATUS_ON)
#define IS_BARO_DTRY(sensor) ((sensor).status & BARO_STATUS_DTRY)

#endif