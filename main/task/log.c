#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"
#include "flight/flight.h"
#include "task/task.h"

struct s_log_task {
	unsigned int time;
	void (*func)();
};

void send_flight_attitude()
{
	anotc_send_imu(flight.imu.accel.raw.x, flight.imu.accel.raw.y, flight.imu.accel.raw.z, flight.imu.gyro.raw.x, flight.imu.gyro.raw.y, flight.imu.gyro.raw.z, 0);
	// anotc_send_quaternion(flight.attitude.q0, flight.attitude.q1, flight.attitude.q2, flight.attitude.q3, 0);
	anotc_send_euler(flight.attitude.roll, flight.attitude.pitch, flight.attitude.yaw, 0);
}

void send_flight_compass()
{
	if (IS_COMPASS_DTRY(flight.compass)) {
		anotc_send_mag(flight.compass.raw.x, flight.compass.raw.y, flight.compass.raw.z, flight.compass.temperature.value, IS_COMPASS_ON(flight.compass));
	}
	if (IS_BARO_READYTOUSE(flight.baro)) {
		anotc_send_alt((int)flight.baro.altitude, 0, 0, 0);
	}
}

void send_system_info()
{
	anotc_send_battery(flight.battery.voltage, flight.battery.current);
}

static struct s_log_task log_task_list[] = {
	{.time=1, .func=send_flight_compass},
	{.time=1, .func=send_flight_attitude},
	{.time=100, .func=send_system_info},
};

static unsigned int log_task_timer = 0;

void log_task()
{
	for (int i = 0; i < sizeof(log_task_list) / sizeof(struct s_log_task); i++)
	{
		if (log_task_timer%log_task_list[i].time==0) {
			log_task_list[i].func();
		}
	}
	log_task_timer++;
}