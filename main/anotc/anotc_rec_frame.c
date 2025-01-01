#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

void anotc_send_gps(unsigned char fix_sta, unsigned char s_num, int lng, int lat, int alt, short n_spe, short s_spe, short d_spe, unsigned char pdop, unsigned char sacc, unsigned char vacc)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_IMU;

	frame.data[frame.len++] = fix_sta;
	frame.data[frame.len++] = s_num;
	anotc_add_int(&frame, lng);
	anotc_add_int(&frame, lat);
	anotc_add_int(&frame, alt);
	anotc_add_short(&frame, n_spe);
	anotc_add_short(&frame, s_spe);
	anotc_add_short(&frame, d_spe);
	frame.data[frame.len++] = pdop;
	frame.data[frame.len++] = sacc;
	frame.data[frame.len++] = vacc;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}