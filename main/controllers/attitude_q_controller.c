#include "flight/flight.h"
#include "misc/config.h"
#include "mixer/mixer.h"
#include "controllers/angle_q_pid.h"
#include "flight/control.h"
#include "clocksource/clocksource.h"
#include "controllers/attitude_q_controller.h"

struct angle_q_pid_param angle_q_pid;

void init_attitude_q_controller()
{
	config_read_float("rol_r_pid.p", &angle_rate_pid[PID_ROLL].kp);
	config_read_float("rol_r_pid.i", &angle_rate_pid[PID_ROLL].ki);
	config_read_float("rol_r_pid.d", &angle_rate_pid[PID_ROLL].kd);
	config_read_ushort("rol_r_pid.d_f", &angle_rate_pid[PID_ROLL].filter.cut_off_freq);
	config_read_float("rol_r_pid.ff", &angle_rate_pid[PID_ROLL].ff);
	if (angle_rate_pid[PID_ROLL].filter.cut_off_freq==0) {
		angle_rate_pid[PID_ROLL].enable_filter = 0;
	} else {
		angle_rate_pid[PID_ROLL].enable_filter = 1;
	}

	config_read_float("pit_r_pid.p", &angle_rate_pid[PID_PITCH].kp);
	config_read_float("pit_r_pid.i", &angle_rate_pid[PID_PITCH].ki);
	config_read_float("pit_r_pid.d", &angle_rate_pid[PID_PITCH].kd);
	config_read_ushort("pit_r_pid.d_f", &angle_rate_pid[PID_PITCH].filter.cut_off_freq);
	config_read_float("pit_r_pid.ff", &angle_rate_pid[PID_PITCH].ff);
	if (angle_rate_pid[PID_PITCH].filter.cut_off_freq==0) {
		angle_rate_pid[PID_PITCH].enable_filter = 0;
	} else {
		angle_rate_pid[PID_PITCH].enable_filter = 1;
	}

	config_read_float("yaw_r_pid.p", &angle_rate_pid[PID_YAW].kp);
	config_read_float("yaw_r_pid.i", &angle_rate_pid[PID_YAW].ki);
	config_read_float("yaw_r_pid.d", &angle_rate_pid[PID_YAW].kd);
	config_read_ushort("yaw_r_pid.d_f", &angle_rate_pid[PID_YAW].filter.cut_off_freq);
	config_read_float("yaw_r_pid.ff", &angle_rate_pid[PID_YAW].ff);
	if (angle_rate_pid[PID_YAW].filter.cut_off_freq==0) {
		angle_rate_pid[PID_YAW].enable_filter = 0;
	} else {
		angle_rate_pid[PID_YAW].enable_filter = 1;
	}

	for (int i=0;i<PID_YAW+1;i++) {
		angle_rate_pid[i].dt = 1.0f/((float)(sys_timer_get()->freq));
		angle_rate_pid[i].filter.freq = sys_timer_get()->freq;
		angle_rate_pid[i].err_limit = 0.3;
	}

	config_read_float("rol_pid.p", &angle_q_pid.K_roll);
	config_read_float("pit_pid.p", &angle_q_pid.K_pitch);
	config_read_float("yaw_pid.p", &angle_q_pid.k_yaw);
}

void attitude_q_controller_update()
{
	float pitch=.0f, roll=.0f, yaw=.0f;
	float _desired_roll_rate=0.0f, _desired_pitch_rate = 0.0f, _desired_yaw_rate =0.0f;

	if (flight.status==FLIGHT_STATUS_MOTOR_TEST)
		return;

	if ((flight.status!=FLIGHT_STATUS_ANGLE_MODE && flight.status!=FLIGHT_STATUS_ANGLE_RATE_MODE)
		|| flight.throttle==0.0f) {
		for (int i=0;i<PID_YAW+1;i++) {
			pid_reset(&angle_rate_pid[i]);
		}

		flight.mixer.motor[0]->value = 0;
		flight.mixer.motor[1]->value = 0;
		flight.mixer.motor[2]->value = 0;
		flight.mixer.motor[3]->value = 0;
		return;
	}

	angle_q_pid.q_current_b_2_w.q0 = flight.attitude.q.q0;
	angle_q_pid.q_current_b_2_w.q1 = flight.attitude.q.q1;
	angle_q_pid.q_current_b_2_w.q2 = flight.attitude.q.q2;
	angle_q_pid.q_current_b_2_w.q3 = flight.attitude.q.q3;
	angle_rate_pid[PID_ROLL].value = flight.imu.gyro.value.x;
	angle_rate_pid[PID_PITCH].value = flight.imu.gyro.value.y;
	angle_rate_pid[PID_YAW].value = flight.imu.gyro.value.z;

	if (flight.throttle>0.2f) {
		if (flight.status==FLIGHT_STATUS_ANGLE_MODE) {
			euler_2_quat(flight.setpoints.roll, flight.setpoints.pitch, flight.attitude.yaw, &angle_q_pid.q_desired_b_2_w);
			_desired_roll_rate = angle_q_pid.output_roll;
			_desired_pitch_rate = angle_q_pid.output_pitch;
			_desired_yaw_rate = angle_q_pid.output_yaw;
		} else {
			_desired_roll_rate = flight.setpoints.roll;
			_desired_pitch_rate = flight.setpoints.pitch;
			_desired_yaw_rate = flight.setpoints.yaw;
		}
	
		pid_calculate(&angle_rate_pid[PID_ROLL], _desired_roll_rate);
		pid_calculate(&angle_rate_pid[PID_PITCH],_desired_pitch_rate);
		pid_calculate(&angle_rate_pid[PID_YAW],_desired_yaw_rate);
		roll = angle_rate_pid[PID_ROLL].output * 0.70710678f;
		pitch = angle_rate_pid[PID_PITCH].output * 0.70710678f;
		yaw = angle_rate_pid[PID_YAW].output;
	} else {
		angle_rate_pid[PID_ROLL].prev_value = flight.imu.gyro.value.x;
		angle_rate_pid[PID_PITCH].prev_value = flight.imu.gyro.value.y;
		angle_rate_pid[PID_YAW].prev_value = flight.imu.gyro.value.z;
	}

	update_flight_mixer(roll, pitch, yaw);
}