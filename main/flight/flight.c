#include "flight/flight.h"

struct flight flight;

void init_flight()
{
	init_imu(&flight.imu);
}

void flight_read_data()
{
	if (flight.imu.status==IMU_STATUS_ON) {
		flight.imu.read(&flight.imu);
		imu_filter(&flight.imu);
	}
	if (flight.compass.status==COMPASS_STATUS_ON) {
		flight.compass.read(&flight.compass);
	}
	if (flight.baro.status==BARO_STATUS_ON) {
		flight.baro.read(&flight.baro);
	}
}