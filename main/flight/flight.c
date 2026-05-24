#include "flight/flight.h"
#include "flight/attitude.h"
#include "flight/altitude.h"
#include "mixer/mixer.h"
#include "flight/control.h"
#include "rc/rc.h"
#include "sensors/voltagemeter.h"

struct flight flight;

void init_flight()
{
	flight.status = FLIGHT_STATUS_SELFTEST;
	init_mixer();
	init_imu(&flight.imu);
	init_baro(&flight.baro);
	init_magnetometer(&flight.magnetometer);
	init_attitude();
	init_control();
	init_rc();
	init_voltagemeter();
}

void flight_read_data()
{
	if (flight.imu.status==IMU_STATUS_ON) {
		flight.imu.read(&flight.imu);
		imu_calibration(&flight.imu);
		imu_filter(&flight.imu);
	}
	if (IS_MAGNETOMETER_ON(flight.magnetometer)) {
		flight.magnetometer.read(&flight.magnetometer);
		magnetometer_filter(&flight.magnetometer);
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
	rc_input(&flight.rc);
	if (!IS_BARO_DTRY(flight.baro) && !IS_MAGNETOMETER_DTRY(flight.magnetometer)) {
		voltagemeter_read(&flight.battery.voltage);
	}
	control_update();
	mixer_output(&flight.mixer);
}