#ifndef SENSORS_COMPASS_H
#define SENSORS_COMPASS_H

#include "bus/bus.h"

#define COMPASS_STATUS_ON 0x1
#define COMPASS_STATUS_DTRY 0x02

struct compass_sensor {
	char *name;
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
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct compass_sensor *sensor);
	unsigned short freq;
};

#define IS_COMPASS_ON(sensor) ((sensor).status & COMPASS_STATUS_ON)
#define IS_COMPASS_DTRY(sensor) ((sensor).status & COMPASS_STATUS_DTRY)

void init_compass(struct compass_sensor *sensor);
void compass_filter(struct compass_sensor *sensor);
#endif