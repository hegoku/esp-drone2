#include <math/math.h>
#include "anotc/anotc_official_frame.h"

#include <esp_system.h>
#include "flight/flight.h"

char* start_calibrate_gyro(unsigned char *param)
{
	if (flight.status!=FLIGHT_STATUS_READY) {
		return "Flight isn't in ready status";
	}
	flight.status = FLIGHT_STATUS_CALIBRATION_GYRO;
	return 0;
}

char* start_calibrate_accel(unsigned char *param)
{
	if (param[0]==0) {
		if (flight.status!=FLIGHT_STATUS_READY) {
			return "Flight isn't in ready status";
		}
		flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL;
	} else {
		if (flight.status != FLIGHT_STATUS_CALIBRATION_ACCEL) {
			return "Flight isn't in accelerator calibration status";
		}
		switch (param[0]) {
			case 'U':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_UP;
				break;
			case 'D':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_DOWN;
				break;
			case 'F':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_FORWARD;
				break;
			case 'B':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_BACKWARD;
				break;
			case 'L':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_LEFT;
				break;
			case 'R':
				flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL_RIGHT;
				break;
			default:
				return "Calibration direction is wrong";
				break;
		}
	}
	return 0;
}

char* reboot(unsigned char *param)
{
	esp_restart();
	return 0;
}

char* toggle_test_mode(unsigned char *param)
{
	if (param[0]==1) {
		if (flight.status!=FLIGHT_STATUS_READY) {
			return "Flight isn't in ready status1";
		}
		flight.status = FLIGHT_STATUS_MOTOR_TEST;
	} else {
		if (flight.status!=FLIGHT_STATUS_MOTOR_TEST) {
			return "Flight isn't in motor test status";
		}
		flight.status = FLIGHT_STATUS_READY;
		flight.mixer->motor[0].value = 0;
		flight.mixer->motor[1].value = 0;
		flight.mixer->motor[2].value = 0;
		flight.mixer->motor[3].value = 0;
	}
	return 0;
}

char* test_mode_throttle(unsigned char *param)
{
	if (flight.status!=FLIGHT_STATUS_MOTOR_TEST) {
		return "Flight isn't in motor test status";
	}
	flight.mixer->motor[0].value = constrain((param[0] | (((unsigned short)param[1])<<8)) - 1000, 0, 1000);
	flight.mixer->motor[1].value = constrain((param[2] | (((unsigned short)param[3])<<8)) - 1000, 0, 1000);
	flight.mixer->motor[2].value = constrain((param[4] | (((unsigned short)param[5])<<8)) - 1000, 0 ,1000);
	flight.mixer->motor[3].value = constrain((param[6] | (((unsigned short)param[7])<<8)) - 1000, 0, 1000);
	return 0;
}

static struct anotc_cmd_info cmd_list[] = {
	{
		.cid = ANOTC_CMD_CALIBRATE_GYRO,
		.handle = start_calibrate_gyro
	},
	{
		.cid = ANOTC_CMD_CALIBRATE_ACCEL,
		.handle = start_calibrate_accel
	},
	{
		.cid = ANOTC_CMD_REBOOT,
		.handle = reboot
	},
	{
		.cid = ANOTC_CMD_TOGGLE_MOTOR_TEST_STATUS,
		.handle = toggle_test_mode
	},
	{
		.cid = ANOTC_CMD_MOTOR_TEST_THROLLE,
		.handle = test_mode_throttle
	}
};

#define ANOTC_CMD_LIST_SIZE sizeof(cmd_list)/sizeof(struct anotc_cmd_info)