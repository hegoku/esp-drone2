#include "anotc/anotc_official_frame.h"

#include "flight/flight.h"

char* start_calibrate_gyro(unsigned char *param)
{
	if (flight.status!=FLIGHT_STATUS_READY) {
		return "Flight doesn't in ready status";
	}
	flight.status = FLIGHT_STATUS_CALIBRATION_GYRO;
	return 0;
}

static struct anotc_cmd_info cmd_list[] = {
	{
		.cid = ANOTC_CMD_CALIBRATE_GYRO,
		.handle = start_calibrate_gyro
	}
};

#define ANOTC_CMD_LIST_SIZE sizeof(cmd_list)/sizeof(struct anotc_cmd_info)