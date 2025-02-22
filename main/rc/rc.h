#ifndef RC_RC_H
#define RC_RC_H

enum rc_status
{
	RC_STATUS_UNCONNECTED,
	RC_STATUS_CONNECTED,
	RC_STATUS_TIMEOUT,
};

#define RC_CHANNEL_AUX1 4
#define RC_CHANNEL_AUX2 5
#define RC_CHANNEL_AUX3 6
#define RC_CHANNEL_AUX4 7
#define RC_CHANNEL_AUX5 8
#define RC_CHANNEL_AUX6 9
#define RC_CHANNEL_AUX7 10
#define RC_CHANNEL_AUX8 11
#define RC_CHANNEL_AUX9 12
#define RC_CHANNEL_AUX10 13
#define RC_CHANNEL_CNT RC_CHANNEL_AUX10

struct rc;

struct rc_protocol {
	char *name;
	void (*init)();
	void (*read)(struct rc *rc);
};

struct rc {
	enum rc_status status;
	struct rc_protocol *protocol;
	unsigned short channel[14];

	short throttle;
	short roll;
	short pitch;
	short yaw;
};

void init_rc();
void rc_input(struct rc *rc);

#endif