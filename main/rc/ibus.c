#include <string.h>
#include "rc/ibus.h"
#include "misc/util.h"

#define MAX_CHANNEL_CNT 14

static unsigned short channel_data1[MAX_CHANNEL_CNT];
static unsigned short channel_data2[MAX_CHANNEL_CNT];
static unsigned short *read_pointer = 0;
static unsigned short *write_pointer = channel_data2;
static unsigned int ts;

void ibus_decode(unsigned char *data, int count)
{
	static unsigned char step = 0;
	static unsigned int _ck_sum = 0;
	static unsigned short channel_tmp[MAX_CHANNEL_CNT] = {0};
					
	for (int i = 0; i < count;i++) {
		switch (step)
		{
		case 0:
			_ck_sum = 0;
			if (data[i] == 0x20)
			{
				step++;
				_ck_sum += data[i];
			}
			break;
		case 1:
			if (data[i]==0x40) {
				step++;
				_ck_sum += data[i];
			} else {
				step = 0;
			}
			break;
		case 2:
			_ck_sum += data[i];
			channel_tmp[0] = data[i];
			step++;
			break;
		case 3:
			_ck_sum += data[i];
			channel_tmp[0] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 4:
			_ck_sum += data[i];
			channel_tmp[1] = data[i];
			step++;
			break;
		case 5:
			_ck_sum += data[i];
			channel_tmp[1] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 6:
			_ck_sum += data[i];
			channel_tmp[2] = data[i];
			step++;
			break;
		case 7:
			_ck_sum += data[i];
			channel_tmp[2] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 8:
			_ck_sum += data[i];
			channel_tmp[3] = data[i];
			step++;
			break;
		case 9:
			_ck_sum += data[i];
			channel_tmp[3] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 10:
			_ck_sum += data[i];
			channel_tmp[4] = data[i];
			step++;
			break;
		case 11:
			_ck_sum += data[i];
			channel_tmp[4] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 12:
			_ck_sum += data[i];
			channel_tmp[5] = data[i];
			step++;
			break;
		case 13:
			_ck_sum += data[i];
			channel_tmp[5] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 14:
			_ck_sum += data[i];
			channel_tmp[6] = data[i];
			step++;
			break;
		case 15:
			_ck_sum += data[i];
			channel_tmp[6] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 16:
			_ck_sum += data[i];
			channel_tmp[7] = data[i];
			step++;
			break;
		case 17:
			_ck_sum += data[i];
			channel_tmp[7] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 18:
			_ck_sum += data[i];
			channel_tmp[8] = data[i];
			step++;
			break;
		case 19:
			_ck_sum += data[i];
			channel_tmp[8] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 20:
			_ck_sum += data[i];
			channel_tmp[9] = data[i];
			step++;
			break;
		case 21:
			_ck_sum += data[i];
			channel_tmp[9] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 22:
			_ck_sum += data[i];
			channel_tmp[10] = data[i];
			step++;
			break;
		case 23:
			_ck_sum += data[i];
			channel_tmp[10] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 24:
			_ck_sum += data[i];
			channel_tmp[11] = data[i];
			step++;
			break;
		case 25:
			_ck_sum += data[i];
			channel_tmp[11] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 26:
			_ck_sum += data[i];
			channel_tmp[12] = data[i];
			step++;
			break;
		case 27:
			_ck_sum += data[i];
			channel_tmp[12] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 28:
			_ck_sum += data[i];
			channel_tmp[13] = data[i];
			step++;
			break;
		case 29:
			_ck_sum += data[i];
			channel_tmp[13] |= (data[i] & 0xF)<<8;
			step++;
			break;
		case 30:
			_ck_sum = 0xFFFF - _ck_sum;
			if ((_ck_sum & 0xFF) == data[i])
			{
				step++;
			}
			else
			{
				step = 0;
			}
			break;
		case 31:
			if ((_ck_sum>>8)==data[i]) {
				if (read_pointer==0) {
					read_pointer = channel_data1;
					write_pointer = channel_data2;
				}
				memcpy(write_pointer, channel_tmp, sizeof(channel_tmp));
				unsigned short *tmp = write_pointer;
				write_pointer = read_pointer;
				read_pointer = tmp;
				ts = get_timestamp_ms();
			}
			step = 0;
			break;
		}
	}
}

void ibus_read(struct rc *rc)
{
	unsigned short *pointer = read_pointer;
	unsigned int current_ts = get_timestamp_ms();
	if (pointer == 0)
	return;
	if (current_ts - ts > 20 && rc->status == RC_STATUS_CONNECTED)
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

void init_ibus()
{
	ibus_hw_init();
	write_pointer = 0;
	read_pointer = 0;
}

struct rc_protocol ibus = {
	.name="IBUS",
	.init=init_ibus,
	.read=ibus_read
};