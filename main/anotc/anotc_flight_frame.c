#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"

void anotc_send_imu(short acc_x, short acc_y, short acc_z, short gyr_x, short gyr_y, short gry_z, unsigned char shock)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_IMU;

	anotc_add_short(&frame, acc_x);
	anotc_add_short(&frame, acc_y);
	anotc_add_short(&frame, acc_z);
	anotc_add_short(&frame, gyr_x);
	anotc_add_short(&frame, gyr_y);
	anotc_add_short(&frame, gry_z);
	frame.data[frame.len++] = shock;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_mag_baro_temp(short mag_x, short mag_y, short mag_z, int alt_bar, float temp, unsigned char bar_sta, unsigned char mag_sta)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_MAG_BAR_TEMP;

	anotc_add_short(&frame, mag_x);
	anotc_add_short(&frame, mag_y);
	anotc_add_short(&frame, mag_z);
	anotc_add_int(&frame, alt_bar);
	anotc_add_short(&frame, (short)(temp*10));
	frame.data[frame.len++] = bar_sta;
	frame.data[frame.len++] = mag_sta;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_euler(float roll, float pitch, float yaw, unsigned char fusion)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_EULER;

	anotc_add_short(&frame, (short)(roll * 100));
	anotc_add_short(&frame, (short)(pitch*100));
	anotc_add_short(&frame, (short)(yaw*100));
	frame.data[frame.len++] = fusion;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_quaternion(float q1, float q2, float q3, float q4, unsigned char fusion)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_EULER;

	anotc_add_short(&frame, (short)(q1 * 10000));
	anotc_add_short(&frame, (short)(q2*10000));
	anotc_add_short(&frame, (short)(q3*10000));
	anotc_add_short(&frame, (short)(q4*10000));
	frame.data[frame.len++] = fusion;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_alt(int alt_fu, int alt_add, unsigned char sta)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_ALT;

	anotc_add_int(&frame, alt_fu);
	anotc_add_int(&frame, alt_add);
	frame.data[frame.len++] = sta;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_target_attitude(float roll, float pitch, float yaw)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_TARGET_ATTITUDE;

	anotc_add_short(&frame, (short)(roll * 100));
	anotc_add_short(&frame, (short)(pitch*100));
	anotc_add_short(&frame, (short)(yaw*100));
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_battery(float voltage, float current)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_BATTERY;

	anotc_add_ushort(&frame, (short)(voltage * 100));
	anotc_add_ushort(&frame, (short)(current*100));
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}