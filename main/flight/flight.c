#include "flight/flight.h"
#include "flight/attitude.h"

struct flight flight;

void init_flight()
{
	init_imu(&flight.imu);
	init_attitude();
}

void flight_read_data()
{
	if (flight.imu.status==IMU_STATUS_ON) {
		flight.imu.read(&flight.imu);
		imu_filter(&flight.imu);
	}
	if (IS_COMPASS_ON(flight.compass)) {
		flight.compass.read(&flight.compass);
	}
	if (flight.baro.status==BARO_STATUS_ON) {
		flight.baro.read(&flight.baro);
	}
}

void flight_update()
{
	calculate_attitude();
}