#include "anotc/anotc_official_frame.h"

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

static struct anotc_cmd_info cmd_list[] = {
	{
		.cid = ANOTC_CMD_CALIBRATE_GYRO,
		.handle = start_calibrate_gyro
	},
	{
		.cid = ANOTC_CMD_CALIBRATE_ACCEL,
		.handle = start_calibrate_accel
	}
};

#define ANOTC_CMD_LIST_SIZE sizeof(cmd_list)/sizeof(struct anotc_cmd_info)