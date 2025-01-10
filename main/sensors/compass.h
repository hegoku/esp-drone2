#ifndef SENSORS_COMPASS_H
#define SENSORS_COMPASS_H

#include "bus/bus.h"

enum compass_status {
	COMPASS_STATUS_OFF,
	COMPASS_STATUS_ON
};

struct compass_sensor {
	char *name;
	enum compass_status status;
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
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct compass_sensor *sensor);
};

#endif