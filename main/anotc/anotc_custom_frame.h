#ifndef ANOTC_CUSTOM_FRAME_H
#define ANOTC_CUSTOM_FRAME_H

#define ANOTC_FRAME_CUSTOM_SYSTEM_INFO 0xF1
#define ANOTC_FRAME_CUSTOM_PID 0xF2

extern unsigned char pid_log_type;

void anotc_send_system_info();
void anotc_send_pid();
void anotc_send_var(float gyro_x, float gyro_y, float gyrp_z, float accel_x, float accel_y, float accel_z, float magnitude, unsigned char stationary);
#endif