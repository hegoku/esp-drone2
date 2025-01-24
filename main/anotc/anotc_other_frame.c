#include <string.h>
#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

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