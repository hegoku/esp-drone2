#include <string.h>
#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

void anotc_send_frame_check(unsigned char id, unsigned char sc, unsigned char ac, unsigned char code, char *msg)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_FRAME_CHECK;

	frame.data[frame.len++] = id;
	frame.data[frame.len++] = sc;
	frame.data[frame.len++] = ac;
	frame.data[frame.len++] = code;
	if (code) {
		anotc_add_string(&frame, msg);
	}
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_log_string(enum anotc_log_string_color color, char *buf, int len)
{
	struct anotc_frame frame;
	int send_len = 0;
	int start = 0;
	while (len > 0)
	{
		send_len = 256 > len ? len : 256;

		PREPARE_ANOTC_FRAME(frame);
		frame.fun = ANOTC_FRAME_LOG_STRING;

		frame.data[frame.len++] = color;
		frame.len += send_len;

		memcpy(frame.data+1+start, buf+start, send_len);
		anotc_add_checksum(&frame);
		_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);

		start += send_len;
		len -= send_len;
	}
}