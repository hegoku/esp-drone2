#ifndef ANOTC_OFFICIAL_FRAME_H
#define ANOTC_OFFICIAL_FRAME_H

#define ANOTC_FRAME_FRAME_CHECK 0x0
#define ANOTC_FRAME_IMU 0x1
#define ANOTC_FRAME_MAG 0x2
#define ANOTC_FRAME_EULER 0x3
#define ANOTC_FRAME_QUAT 0x4
#define ANOTC_FRAME_ALT 0x5
#define ANOTC_FRAME_TARGET_ATTITUDE 0xA
#define ANOTC_FRAME_BATTERY 0xD

#define ANOTC_FRAME_PWM 0x20

#define ANOTC_FRAME_GPS 0x30

#define ANOTC_FRAME_RC 0x40

#define ANOTC_FRAME_LOG_STRING 0xA0
#define ANOTC_FRAME_LOG_STRING_NUM 0xA1

enum anotc_log_string_color{
	BLACK,
	RED,
	GREEN
};

void anotc_send_frame_check(unsigned char id, unsigned char sc, unsigned char ac, unsigned char code, char *msg);
void anotc_send_imu(short acc_x, short acc_y, short acc_z, short gyr_x, short gyr_y, short gry_z, unsigned char shock);
void anotc_send_mag(short mag_x, short mag_y, short mag_z, float temp, unsigned char mag_sta);
void anotc_send_euler(float roll, float pitch, float yaw, unsigned char fusion);
void anotc_send_quaternion(float q1, float q2, float q3, float q4, unsigned char fusion);
void anotc_send_alt(int alt_baro, int alt_add, int alt_fu, unsigned char sta);
void anotc_send_battery(float voltage, float current);

void anotc_send_pwm(unsigned short pwm[8]);

void anotc_send_rc(short channel[14]);

void anotc_send_log_string(enum anotc_log_string_color color, char *buf, int len);
#endif