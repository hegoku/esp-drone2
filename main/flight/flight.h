#ifndef FLIGHT_FLIGHT_H
#define FLIGHT_FLIGHT_H

#include "sensors/imu.h"
#include "sensors/barometer.h"

struct flight {
	struct imu_sensor imu;
	struct barometer_sensor baro;
};

extern struct flight flight;
#endif