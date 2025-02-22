#include "anotc/anotc.h"
#include "anotc/anotc_custom_frame.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "flight/control.h"

void anotc_send_system_info()
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CUSTOM_SYSTEM_INFO;

	frame.data[frame.len++] = flight.status;
	frame.data[frame.len++] = flight.system_info.cpu_load;
	anotc_add_float(&frame, flight.battery.voltage);
	anotc_add_float(&frame, flight.battery.current);
	frame.data[frame.len++] = flight.rc.status;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_pid()
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CUSTOM_PID;

	unsigned int time = get_timestamp_ms();

	anotc_add_uint(&frame, time);

	anotc_add_float(&frame, flight.attitude.roll);
	anotc_add_float(&frame, flight.attitude.pitch);
	anotc_add_float(&frame, flight.attitude.yaw);

	anotc_add_float(&frame, flight.imu.gyro.value.x);
	anotc_add_float(&frame, flight.imu.gyro.value.y);
	anotc_add_float(&frame, flight.imu.gyro.value.z);

	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].P);
	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].I);
	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].D);
	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].output);

	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].P);
	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].I);
	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].D);
	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].output);

	anotc_add_float(&frame, angle_rate_pid[PID_YAW].P);
	anotc_add_float(&frame, angle_rate_pid[PID_YAW].I);
	anotc_add_float(&frame, angle_rate_pid[PID_YAW].D);
	anotc_add_float(&frame, angle_rate_pid[PID_YAW].output);

	anotc_add_float(&frame, angle_pid[PID_ROLL].P);
	anotc_add_float(&frame, angle_pid[PID_ROLL].I);
	anotc_add_float(&frame, angle_pid[PID_ROLL].D);
	anotc_add_float(&frame, angle_pid[PID_ROLL].output);

	anotc_add_float(&frame, angle_pid[PID_PITCH].P);
	anotc_add_float(&frame, angle_pid[PID_PITCH].I);
	anotc_add_float(&frame, angle_pid[PID_PITCH].D);
	anotc_add_float(&frame, angle_pid[PID_PITCH].output);

	anotc_add_float(&frame, angle_pid[PID_YAW].P);
	anotc_add_float(&frame, angle_pid[PID_YAW].I);
	anotc_add_float(&frame, angle_pid[PID_YAW].D);
	anotc_add_float(&frame, angle_pid[PID_YAW].output);

	anotc_add_float(&frame, flight.setpoints.throttle);
	anotc_add_float(&frame, flight.setpoints.roll);
	anotc_add_float(&frame, flight.setpoints.pitch);
	anotc_add_float(&frame, flight.setpoints.yaw);

	anotc_add_ushort(&frame, flight.mixer.motor[0]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[1]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[2]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[3]->value);

	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].desired);
	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].desired);
	anotc_add_float(&frame, angle_rate_pid[PID_YAW].desired);

	anotc_add_short(&frame, (short int)flight.imu.accel.value.x);
	anotc_add_short(&frame, (short int)flight.imu.accel.value.x);
	anotc_add_short(&frame, (short int)flight.imu.accel.value.x);

	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char*)(&frame), ANOTC_V8_HEAD_SIZE + frame.len + 2);
}