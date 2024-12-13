#ifndef SENSORS_BAROMETER_H
#define SENSORS_BAROMETER_H

#include "bus/bus.h"

enum baro_status {
	BARO_STATUS_OFF,
	BARO_STATUS_ON
};

struct barometer_sensor {
	char *name;
	enum baro_status status;
	short pressure;
	short temperature;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct barometer_sensor *sensor);
};

#endif