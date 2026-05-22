#ifndef CONTROLLER_ATTITUDE_CONTROLLER_H
#define CONTROLLER_ATTITUDE_CONTROLLER_H

#include "math/quaternion.h"

struct angle_q_pid_param {
	struct quaternion q_desired_b_2_w;
	struct quaternion q_current_b_2_w;
	float K_roll;
	float K_pitch;
	float k_yaw;
	struct quaternion q_error_b;
	float output_roll;
	float output_pitch;
	float output_yaw;
};

void angle_q_pid(struct angle_q_pid_param *p);
#endif