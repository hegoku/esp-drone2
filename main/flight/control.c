#include <math.h>
#include "flight/control.h"
#include "misc/pid.h"
#include "misc/config.h"
#include "clocksource/clocksource.h"
#include "flight/flight.h"
#include "math/math.h"

#define PID_ROLL 0
#define PID_PITCH 1
#define PID_YAW 2

struct pid_data angle_rate_pid[3];
struct pid_data angle_pid[3];

void init_control()
{
	config_read_float("", &angle_rate_pid[PID_ROLL].kp);
	config_read_float("", &angle_rate_pid[PID_PITCH].kp);
	config_read_float("", &angle_rate_pid[PID_YAW].kp);

	config_read_float("", &angle_rate_pid[PID_ROLL].ki);
	config_read_float("", &angle_rate_pid[PID_PITCH].ki);
	config_read_float("", &angle_rate_pid[PID_YAW].ki);

	config_read_float("", &angle_rate_pid[PID_ROLL].kd);
	config_read_float("", &angle_rate_pid[PID_PITCH].kd);
	config_read_float("", &angle_rate_pid[PID_YAW].kd);

	config_read_float("", &angle_pid[PID_ROLL].kp);
	config_read_float("", &angle_pid[PID_PITCH].kp);
	config_read_float("", &angle_pid[PID_YAW].kp);

	config_read_float("", &angle_pid[PID_ROLL].ki);
	config_read_float("", &angle_pid[PID_PITCH].ki);
	config_read_float("", &angle_pid[PID_YAW].ki);

	config_read_float("", &angle_pid[PID_ROLL].kd);
	config_read_float("", &angle_pid[PID_PITCH].kd);
	config_read_float("", &angle_pid[PID_YAW].kd);

	for (int i=0;i<PID_YAW+1;i++) {
		angle_rate_pid[i].dt = 1.0/((float)(sys_timer_get()->freq));
		angle_pid[i].dt = 1.0/((float)(sys_timer_get()->freq));
	}
}

void control_update()
{
	short pitch=0, roll=0, yaw=0;
	float _desired_roll_rate=0, _desired_pitch_rate = 0, _desired_yaw_rate =0;

	if (flight.status!=FLIGHT_STATUS_ANGLE_MODE && flight.status!=FLIGHT_STATUS_ANGLE_RATE_MODE
		&& flight.throttle==0) {
		for (int i=0;i<PID_YAW+1;i++) {
			pid_reset(&angle_rate_pid[i]);
			pid_reset(&angle_pid[i]);
		}
		return;

		flight.mixer.motor[0]->value = flight.throttle;
		flight.mixer.motor[1]->value = flight.throttle;
		flight.mixer.motor[2]->value = flight.throttle;
		flight.mixer.motor[3]->value = flight.throttle;
	}

	if (flight.status==FLIGHT_STATUS_ANGLE_MODE) {
		pid_calculate(&angle_pid[PID_ROLL], 0);
		pid_calculate(&angle_pid[PID_PITCH], 0);
		_desired_roll_rate = angle_pid[PID_ROLL].output;
		_desired_pitch_rate = angle_pid[PID_PITCH].output;
		_desired_yaw_rate = 0;
	} else {
		_desired_roll_rate = 0;
		_desired_pitch_rate = 0;
		_desired_yaw_rate = 0;
	}

	pid_calculate(&angle_rate_pid[PID_ROLL], _desired_roll_rate);
	pid_calculate(&angle_rate_pid[PID_PITCH],_desired_pitch_rate);
	pid_calculate(&angle_rate_pid[PID_YAW],_desired_yaw_rate);
	roll = (short)round(angle_rate_pid[PID_ROLL].output);
	pitch = (short)round(angle_rate_pid[PID_PITCH].output);
	yaw = (short)round(angle_rate_pid[PID_YAW].output);

	flight.mixer.motor[0]->value = constrain(flight.throttle + roll + pitch + yaw, FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[1]->value = constrain(flight.throttle + roll - pitch - yaw, FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[2]->value = constrain(flight.throttle - roll + pitch - yaw, FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[3]->value = constrain(flight.throttle - roll - pitch + yaw, FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
}