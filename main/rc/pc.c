#include <string.h>
#include "rc/rc.h"
#include "rc/pc.h"
#include "misc/util.h"

#define MAX_CHANNEL_CNT 6

static unsigned short channel_data1[MAX_CHANNEL_CNT];
static unsigned short channel_data2[MAX_CHANNEL_CNT];
static unsigned short *read_pointer = 0;
static unsigned short *write_pointer = channel_data2;
static unsigned int ts;

void pc_rc_receive(unsigned short *channel)
{
	if (read_pointer==0) {
		read_pointer = channel_data1;
		write_pointer = channel_data2;
	}
	memcpy(write_pointer, channel, sizeof(channel_data1));
	unsigned short *tmp = write_pointer;
	write_pointer = read_pointer;
	read_pointer = tmp;
	ts = get_timestamp_ms();
}

void pc_read(struct rc *rc)
{
	unsigned short *pointer = read_pointer;
	unsigned int current_ts = get_timestamp_ms();
	if (pointer == 0)
	return;
	if (current_ts - ts > 50 && rc->status == RC_STATUS_CONNECTED)
	{
		rc->status = RC_STATUS_TIMEOUT;
		for (int i = 0; i < MAX_CHANNEL_CNT; i++)
		{
			rc->channel[i] = 0;
		}
	}
	else
	{
		for (int i = 0; i < MAX_CHANNEL_CNT; i++)
		{
			rc->channel[i] = pointer[i];
		}
		rc->status = RC_STATUS_CONNECTED;
	}
}

void init_pc()
{
	write_pointer = 0;
	read_pointer = 0;
}

struct rc_protocol pc = {
	.name="PC",
	.init=init_pc,
	.read=pc_read
};