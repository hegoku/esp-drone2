#include "anotc/anotc.h"
#include "anotc/anotc_custom_frame.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "flight/control.h"

unsigned char pid_log_type = 0;

void anotc_send_system_info()
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CUSTOM_SYSTEM_INFO;

	frame.data[frame.len++] = flight.status;
	frame.data[frame.len++] = flight.system_info.cpu_load;
	anotc_add_ushort(&frame, (unsigned short)(flight.battery.voltage*100));
	anotc_add_ushort(&frame, (unsigned short)(flight.battery.current*100));
	frame.data[frame.len++] = flight.rc.status;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_pid()
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CUSTOM_PID;

	unsigned char type = PID_ROLL;
	if (pid_log_type==1) {
		type=PID_PITCH;
	} else if (pid_log_type==2) {
		type=PID_YAW;
	}
	anotc_add_uint(&frame, (unsigned int)(flight.system_info.time/1000ULL));

	anotc_add_float(&frame, flight.imu.accel.value.x);
	anotc_add_float(&frame, flight.imu.accel.value.y);
	anotc_add_float(&frame, flight.imu.accel.value.z);

	frame.data[frame.len++] = (char)(flight.attitude.roll);
	frame.data[frame.len++] = (char)(flight.attitude.pitch);
	frame.data[frame.len++] = (char)(flight.attitude.yaw);

	anotc_add_float(&frame, flight.imu.gyro.value.x);
	anotc_add_float(&frame, flight.imu.gyro.value.y);
	anotc_add_float(&frame, flight.imu.gyro.value.z);

	anotc_add_ushort(&frame, (unsigned short)(flight.setpoints.throttle*1000));
	frame.data[frame.len++] = (char)(flight.setpoints.roll);
	frame.data[frame.len++] = (char)(flight.setpoints.pitch);
	frame.data[frame.len++] = (char)(flight.setpoints.yaw);

	anotc_add_ushort(&frame, flight.mixer.motor[0]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[1]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[2]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[3]->value);

	anotc_add_short(&frame, (short)(angle_rate_pid[type].desired));

	anotc_add_short(&frame, (short)(angle_rate_pid[type].P*1000));
	anotc_add_short(&frame, (short)(angle_rate_pid[type].I*1000));
	anotc_add_short(&frame, (short)(angle_rate_pid[type].D*1000));
	anotc_add_short(&frame, (short)(angle_rate_pid[type].output*1000));

	anotc_add_short(&frame, (short)(angle_pid[type].P));
	anotc_add_short(&frame, (short)(angle_pid[type].I));
	anotc_add_short(&frame, (short)(angle_pid[type].D));
	anotc_add_short(&frame, (short)(angle_pid[type].output));

	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char*)(&frame), ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_var(float gyro_x, float gyro_y, float gyrp_z, float accel_x, float accel_y, float accel_z, float magnitude, unsigned char stationary)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = 0xF3;

	anotc_add_float(&frame, gyro_x);
	anotc_add_float(&frame, gyro_y);
	anotc_add_float(&frame, gyrp_z);

	anotc_add_float(&frame, accel_x);
	anotc_add_float(&frame, accel_y);
	anotc_add_float(&frame, accel_z);

	anotc_add_float(&frame, magnitude);
	frame.data[frame.len++] = stationary;

	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char*)(&frame), ANOTC_V8_HEAD_SIZE + frame.len + 2);
}