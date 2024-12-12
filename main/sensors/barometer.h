#ifndef SENSORS_BAROMETER_H
#define SENSORS_BAROMETER_H

#include "bus/bus.h"

struct barometer_sensor {
	char *name;
	short pressure;
	short temperature;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct barometer_sensor *sensor);
};

#endif