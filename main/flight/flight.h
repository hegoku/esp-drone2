#ifndef FLIGHT_FLIGHT_H
#define FLIGHT_FLIGHT_H

#include "sensors/imu.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"

struct flight {
	struct imu_sensor imu;
	struct barometer_sensor baro;
	struct compass_sensor compass;
};

extern struct flight flight;

void init_flight();
void flight_read_data();
#endif