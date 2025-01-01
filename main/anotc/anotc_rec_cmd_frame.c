#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

void anotc_send_rc(short channel[14])
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_RC;

	for (int i = 0; i < 14;i++) {
		anotc_add_short(&frame, channel[i]);
	}
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}