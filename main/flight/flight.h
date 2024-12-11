#ifndef FLIGHT_FLIGHT_H
#define FLIGHT_FLIGHT_H

#include "sensors/imu.h"

struct flight {
	struct imu_sensor *imu;
};

extern struct flight flight;
#endif