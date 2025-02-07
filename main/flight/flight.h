#ifndef FLIGHT_FLIGHT_H
#define FLIGHT_FLIGHT_H

#include "sensors/imu.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "flight/attitude.h"
#include "mixer/mixer.h"

enum flight_status {
	FLIGHT_STATUS_READY,
	FLIGHT_STATUS_ANGLE_RATE_MODE,
	FLIGHT_STATUS_ANGLE_MODE,
	FLIGHT_STATUS_CALIBRATION_ACCEL,
	FLIGHT_STATUS_CALIBRATION_GYRO,
	FLIGHT_STATUS_CALIBRATION_COMPASS,
	FLIGHT_STATUS_MOTOR_TEST
};

struct flight {
	struct imu_sensor imu;
	struct barometer_sensor baro;
	struct compass_sensor compass;

	struct {
		float voltage;
		float current;
	} battery;

	struct flight_attitude attitude;
	float altitude;

	struct {
		unsigned char drone_center_connect;
		float cpu_load;
	} system_info;

	enum flight_status status;

	struct mixer *mixer;
};

extern struct flight flight;

#define FLIGHT_IS_ARMED (flight.status==FLIGHT_STATUS_ANGLE_RATE_MODE || flight.status==FLIGHT_STATUS_ANGLE_MODE)

void init_flight();
void flight_read_data();
void flight_update();
void flight_control();
#endif