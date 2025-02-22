#include "rc/rc.h"
#include "flight/flight.h"
#include "misc/config.h"
#include "math/math.h"

#include "rc/ibus.h"

struct rc_cmd {
	unsigned char channel;
	void (*handler)(unsigned short value);
	unsigned short min;
	unsigned short max;
};

void init_rc()
{
	unsigned char protocol = 0;
	config_read_uchar("rc_protocol", &protocol);
	switch (protocol)
	{
	case 0:
		flight.rc.protocol = &ibus;
		break;
	default:
		flight.rc.protocol = &ibus;
		break;
	}
	flight.rc.protocol->init();
}

#include "rc_cmd_list.c"

void rc_input(struct rc *rc)
{
	rc->protocol->read(rc);
	if (rc->status==RC_STATUS_CONNECTED) {
		rc->throttle = constrain(flight.rc.channel[2] - 1000, 0, 1000);
		rc->roll = constrain(flight.rc.channel[0] - 1500, -500, 500);
		rc->pitch = constrain(flight.rc.channel[1] - 1500,-500, 500);
		rc->yaw = constrain(flight.rc.channel[3] - 1500, -500, 500);

		for (int i = 0; i < sizeof(rc_cmd_list)/sizeof(struct rc_cmd);i++) {
			if (rc->channel[rc_cmd_list[i].channel]>=rc_cmd_list[i].min && rc->channel[rc_cmd_list[i].channel]<=rc_cmd_list[i].max) {
				rc_cmd_list[i].handler(rc->channel[rc_cmd_list[i].channel]);
			}
		}

		if (flight.status == FLIGHT_STATUS_ANGLE_MODE || flight.status == FLIGHT_STATUS_ANGLE_RATE_MODE) {
			flight.setpoints.throttle = ((float)rc->throttle) / 1000.0f;
			flight.throttle = flight.setpoints.throttle;
			if (flight.status == FLIGHT_STATUS_ANGLE_MODE) {
				flight.setpoints.roll = (float)rc->roll * 30.0f / 500.0f;
				flight.setpoints.pitch = (float)rc->pitch * 30.0f / 500.0f;
			} else {
				flight.setpoints.roll = (float)rc->roll * 90.0f / 500.0f;
				flight.setpoints.pitch = (float)rc->pitch * 90.0f / 500.0f;
			}
			flight.setpoints.yaw = (float)rc->yaw * 90.0f / 500.0f;
		}
	} else if (rc->status==RC_STATUS_TIMEOUT) {
		flight.throttle = 0;
		if (flight.status==FLIGHT_STATUS_ANGLE_MODE || flight.status==FLIGHT_STATUS_ANGLE_RATE_MODE) {
			flight.status = FLIGHT_STATUS_READY;
		}
	}
}