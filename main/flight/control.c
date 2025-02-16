#include "flight/control.h"
#include "misc/pid.h"
#include "misc/config.h"
#include "clocksource/clocksource.h"
#include "flight/flight.h"

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
	if (flight.status!=FLIGHT_STATUS_ANGLE_MODE && flight.status!=FLIGHT_STATUS_ANGLE_RATE_MODE) {
		for (int i=0;i<PID_YAW+1;i++) {
			pid_reset(&angle_rate_pid[i]);
			pid_reset(&angle_pid[i]);
		}
		return;
	}
}