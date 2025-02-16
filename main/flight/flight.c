#include "flight/flight.h"
#include "flight/attitude.h"
#include "flight/altitude.h"
#include "mixer/mixer.h"
#include "flight/control.h"

struct flight flight;

void init_flight()
{
	flight.status = FLIGHT_STATUS_READY;
	init_mixer();
	init_imu(&flight.imu);
	init_baro(&flight.baro);
	init_attitude();
}

void flight_read_data()
{
	if (flight.imu.status==IMU_STATUS_ON) {
		flight.imu.read(&flight.imu);
		imu_calibration(&flight.imu);
		imu_filter(&flight.imu);
	}
	if (IS_COMPASS_ON(flight.compass)) {
		flight.compass.read(&flight.compass);
	}
	if (IS_BARO_ON(flight.baro)) {
		flight.baro.read(&flight.baro);
		baro_pressure2altitude(&flight.baro);
		baro_calcuate_altitude(&flight.baro);
	}
}

void flight_update()
{
	calculate_attitude();
	calculate_altitude();
}

void flight_control()
{
	mixer_output(&flight.mixer);
}