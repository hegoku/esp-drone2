#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

void anotc_send_pwm(unsigned short pwm[8])
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_PWM;

	for (int i = 0; i < 8;i++) {
		anotc_add_ushort(&frame, pwm[i]);
	}
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}