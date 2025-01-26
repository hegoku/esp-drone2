#ifndef SENSORS_BAROMETER_H
#define SENSORS_BAROMETER_H

#include "bus/bus.h"

#define BARO_STATUS_ON 0x1
#define BARO_STATUS_DTRY 0x2
#define BARO_STATUS_CALIBRATED 0x4

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
	float actual_altitude;
	float ground_altitude;
	unsigned short calibration_count;
	void *priv;

	struct bus_dev *dev;
	int(*read)(struct barometer_sensor *sensor);
};

#define IS_BARO_ON(sensor) ((sensor).status & BARO_STATUS_ON)
#define IS_BARO_DTRY(sensor) ((sensor).status & BARO_STATUS_DTRY)
#define IS_BARO_CALIBRATED(sensor) ((sensor).status & BARO_STATUS_CALIBRATED)
#define IS_BARO_READYTOUSE(sensor) (((sensor).status & (BARO_STATUS_CALIBRATED|BARO_STATUS_DTRY)) == (BARO_STATUS_CALIBRATED|BARO_STATUS_DTRY))

void init_baro(struct barometer_sensor *sensor);
void baro_pressure2altitude(struct barometer_sensor *sensor);
void baro_calcuate_altitude(struct barometer_sensor *sensor);

#endif