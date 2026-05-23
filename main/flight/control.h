#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "controllers/pid.h"
#include "controllers/angle_q_pid.h"

#define PID_ROLL 0
#define PID_PITCH 1
#define PID_YAW 2

void init_control();
void control_update();

extern struct pid_data angle_rate_pid[3];
extern struct pid_data angle_pid[3];
extern struct angle_q_pid_param angle_q_pid;

#endif