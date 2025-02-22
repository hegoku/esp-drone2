#include <math.h>
#include "flight/control.h"
#include "misc/pid.h"
#include "misc/config.h"
#include "clocksource/clocksource.h"
#include "flight/flight.h"
#include "math/math.h"

struct pid_data angle_rate_pid[3];
struct pid_data angle_pid[3];

void init_control()
{
	config_read_float("rol_r_pid.p", &angle_rate_pid[PID_ROLL].kp);
	config_read_float("rol_r_pid.i", &angle_rate_pid[PID_ROLL].ki);
	config_read_float("rol_r_pid.d", &angle_rate_pid[PID_ROLL].kd);
	config_read_ushort("rol_r_pid.d_f", &angle_rate_pid[PID_ROLL].filter.cut_off_freq);
	if (angle_rate_pid[PID_ROLL].filter.cut_off_freq==0) {
		angle_rate_pid[PID_ROLL].enable_filter = 0;
	} else {
		angle_rate_pid[PID_ROLL].enable_filter = 1;
	}

	config_read_float("pit_r_pid.p", &angle_rate_pid[PID_PITCH].kp);
	config_read_float("pit_r_pid.i", &angle_rate_pid[PID_PITCH].ki);
	config_read_float("pit_r_pid.d", &angle_rate_pid[PID_PITCH].kd);
	config_read_ushort("pit_r_pid.d_f", &angle_rate_pid[PID_PITCH].filter.cut_off_freq);
	if (angle_rate_pid[PID_PITCH].filter.cut_off_freq==0) {
		angle_rate_pid[PID_PITCH].enable_filter = 0;
	} else {
		angle_rate_pid[PID_PITCH].enable_filter = 1;
	}

	config_read_float("yaw_r_pid.p", &angle_rate_pid[PID_YAW].kp);
	config_read_float("yaw_r_pid.i", &angle_rate_pid[PID_YAW].ki);
	config_read_float("yaw_r_pid.d", &angle_rate_pid[PID_YAW].kd);
	config_read_ushort("yaw_r_pid.d_f", &angle_rate_pid[PID_YAW].filter.cut_off_freq);
	if (angle_rate_pid[PID_YAW].filter.cut_off_freq==0) {
		angle_rate_pid[PID_YAW].enable_filter = 0;
	} else {
		angle_rate_pid[PID_YAW].enable_filter = 1;
	}

	config_read_float("rol_pid.p", &angle_pid[PID_ROLL].kp);
	config_read_float("rol_pid.i", &angle_pid[PID_ROLL].ki);
	config_read_float("rol_pid.d", &angle_pid[PID_ROLL].kd);
	config_read_ushort("rol_pid.d_f", &angle_pid[PID_ROLL].filter.cut_off_freq);
	if (angle_pid[PID_ROLL].filter.cut_off_freq==0) {
		angle_pid[PID_ROLL].enable_filter = 0;
	} else {
		angle_pid[PID_ROLL].enable_filter = 1;
	}

	config_read_float("pit_pid.p", &angle_pid[PID_PITCH].kp);
	config_read_float("pit_pid.i", &angle_pid[PID_PITCH].ki);
	config_read_float("pit_pid.d", &angle_pid[PID_PITCH].kd);
	config_read_ushort("pit_pid.d_f", &angle_pid[PID_PITCH].filter.cut_off_freq);
	if (angle_pid[PID_PITCH].filter.cut_off_freq==0) {
		angle_pid[PID_PITCH].enable_filter = 0;
	} else {
		angle_pid[PID_PITCH].enable_filter = 1;
	}

	config_read_float("yaw_pid.p", &angle_pid[PID_YAW].kp);
	config_read_float("yaw_pid.i", &angle_pid[PID_YAW].ki);
	config_read_float("yaw_pid.d", &angle_pid[PID_YAW].kd);
	config_read_ushort("yaw_pid.d_f", &angle_pid[PID_YAW].filter.cut_off_freq);
	if (angle_pid[PID_YAW].filter.cut_off_freq==0) {
		angle_pid[PID_YAW].enable_filter = 0;
	} else {
		angle_pid[PID_YAW].enable_filter = 1;
	}

	for (int i=0;i<PID_YAW+1;i++) {
		angle_rate_pid[i].dt = 1.0/((float)(sys_timer_get()->freq));
		angle_rate_pid[i].filter.freq = sys_timer_get()->freq;
		angle_pid[i].dt = 1.0 / ((float)(sys_timer_get()->freq));
		angle_pid[i].filter.freq = sys_timer_get()->freq;
	}
}

void control_update()
{
	float pitch=.0f, roll=.0f, yaw=.0f;
	float _desired_roll_rate=0, _desired_pitch_rate = 0, _desired_yaw_rate =0;

	if (flight.status==FLIGHT_STATUS_MOTOR_TEST)
		return;

	if ((flight.status!=FLIGHT_STATUS_ANGLE_MODE && flight.status!=FLIGHT_STATUS_ANGLE_RATE_MODE)
		|| flight.throttle==0) {
		for (int i=0;i<PID_YAW+1;i++) {
			pid_reset(&angle_rate_pid[i]);
			pid_reset(&angle_pid[i]);
		}

		flight.mixer.motor[0]->value = flight.throttle;
		flight.mixer.motor[1]->value = flight.throttle;
		flight.mixer.motor[2]->value = flight.throttle;
		flight.mixer.motor[3]->value = flight.throttle;
		return;
	}

	if (flight.status==FLIGHT_STATUS_ANGLE_MODE) {
		pid_calculate(&angle_pid[PID_ROLL], flight.setpoints.roll);
		pid_calculate(&angle_pid[PID_PITCH], flight.setpoints.pitch);
		_desired_roll_rate = angle_pid[PID_ROLL].output;
		_desired_pitch_rate = angle_pid[PID_PITCH].output;
		_desired_yaw_rate = flight.setpoints.yaw;
	} else {
		_desired_roll_rate = flight.setpoints.roll;
		_desired_pitch_rate = flight.setpoints.pitch;
		_desired_yaw_rate = flight.setpoints.yaw;
	}

	pid_calculate(&angle_rate_pid[PID_ROLL], _desired_roll_rate);
	pid_calculate(&angle_rate_pid[PID_PITCH],_desired_pitch_rate);
	pid_calculate(&angle_rate_pid[PID_YAW],_desired_yaw_rate);
	roll = angle_rate_pid[PID_ROLL].output * 0.707106781f;
	pitch = angle_rate_pid[PID_PITCH].output * 0.707106781f;
	yaw = angle_rate_pid[PID_YAW].output;

	flight.mixer.motor[0]->value = constrain(round((flight.throttle + roll + pitch + yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[1]->value = constrain(round((flight.throttle + roll - pitch - yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[2]->value = constrain(round((flight.throttle - roll + pitch - yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[3]->value = constrain(round((flight.throttle - roll - pitch + yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
}