#include "flight/flight.h"

static void rc_cmd_switch_to_ready(unsigned short value)
{
	if ((flight.status==FLIGHT_STATUS_ANGLE_MODE || flight.status==FLIGHT_STATUS_ANGLE_RATE_MODE)) {
		flight.throttle = 0;
		flight.status = FLIGHT_STATUS_READY;
	}
}

static void rc_cmd_switch_to_angle_status(unsigned short value)
{
	if ((flight.status==FLIGHT_STATUS_READY || flight.status==FLIGHT_STATUS_ANGLE_RATE_MODE) && flight.throttle==0) {
		flight.status = FLIGHT_STATUS_ANGLE_MODE;
	}
}

static void rc_cmd_switch_to_angle_rate_status(unsigned short value)
{
	if ((flight.status==FLIGHT_STATUS_READY || flight.status==FLIGHT_STATUS_ANGLE_MODE) && flight.throttle==0) {
		flight.status = FLIGHT_STATUS_ANGLE_RATE_MODE;
	}
}

struct rc_cmd rc_cmd_list[] = {
	{
		.channel=RC_CHANNEL_AUX2,
		.handler=rc_cmd_switch_to_ready,
		.min=0,
		.max=1000
	},
	{
		.channel=RC_CHANNEL_AUX2,
		.handler=rc_cmd_switch_to_angle_status,
		.min=1001,
		.max=1500
	},
	{
		.channel=RC_CHANNEL_AUX2,
		.handler=rc_cmd_switch_to_angle_rate_status,
		.min=1501,
		.max=2000
	}
};