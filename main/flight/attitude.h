#ifndef FLIGHT_ATTITUDE_H
#define FLIGHT_ATTITUDE_H

struct flight_attitude {
	float q0,q1,q2,q3;
	float roll,pitch,yaw;
};

void init_attitude();
void calculate_attitude();

#endif