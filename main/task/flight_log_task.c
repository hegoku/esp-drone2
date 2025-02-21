#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"
#include "anotc/anotc_custom_frame.h"
#include "flight/flight.h"
#include "task/task.h"

struct s_log_task {
	unsigned int time;
	void (*func)();
};

void send_flight_attitude()
{
	anotc_send_imu((short)(flight.imu.accel.value.x*100.0), (short)(flight.imu.accel.value.y*100.0), (short)(flight.imu.accel.value.z*100.0), (short)(flight.imu.gyro.value.x*100.0), (short)(flight.imu.gyro.value.y*100.0), (short)(flight.imu.gyro.value.z*100.0), 0);
	// anotc_send_quaternion(flight.attitude.q0, flight.attitude.q1, flight.attitude.q2, flight.attitude.q3, 0);
	anotc_send_euler(flight.attitude.roll, flight.attitude.pitch, flight.attitude.yaw, 0);
}

void send_flight_compass()
{
	if (IS_COMPASS_DTRY(flight.compass)) {
		anotc_send_mag(flight.compass.raw.x, flight.compass.raw.y, flight.compass.raw.z, flight.compass.temperature.value, IS_COMPASS_ON(flight.compass));
	}
	if (IS_BARO_READYTOUSE(flight.baro)) {
		anotc_send_alt((int)flight.baro.altitude, 0, (int)flight.altitude, 0);
	}
}

void send_system_info()
{
	anotc_send_system_info();
}

void send_motor()
{
	unsigned short pwm[8] = {0,0,0,0,0,0,0,0};
	pwm[0] = flight.mixer.motor[0]->value;
	pwm[1] = flight.mixer.motor[1]->value;
	pwm[2] = flight.mixer.motor[2]->value;
	pwm[3] = flight.mixer.motor[3]->value;
	anotc_send_pwm(pwm);

	anotc_send_rc((short*)&flight.rc.channel);
}

static struct s_log_task log_task_list[] = {
	{.time=1, .func=send_flight_compass},
	{.time=10, .func=send_flight_attitude},
	{.time=100, .func=send_system_info},
	{.time=16, .func=send_motor},
};

static unsigned int log_task_timer = 0;

void flight_log_task()
{
	if (flight.system_info.drone_center_connect) {
		for (int i = 0; i < sizeof(log_task_list) / sizeof(struct s_log_task); i++)
		{
			if (log_task_timer%log_task_list[i].time==0) {
				log_task_list[i].func();
			}
		}
	}
	log_task_timer++;
}