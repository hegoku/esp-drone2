#ifndef FLIGHT_ATTITUDE_H
#define FLIGHT_ATTITUDE_H

#include "math/quaternion.h"

struct flight_attitude {
	struct quaternion q;
	float roll,pitch,yaw;
};

void init_attitude();
void calculate_attitude();

#endif