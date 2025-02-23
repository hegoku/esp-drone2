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

	// unsigned int time = get_timestamp_ms();

	// anotc_add_uint(&frame, time);

	anotc_add_short(&frame, (short)(flight.imu.accel.value.x*100));
	anotc_add_short(&frame, (short)(flight.imu.accel.value.y*100));
	anotc_add_short(&frame, (short)(flight.imu.accel.value.z*100));

	frame.data[frame.len++] = (char)(flight.attitude.roll);
	frame.data[frame.len++] = (char)(flight.attitude.pitch);
	frame.data[frame.len++] = (char)(flight.attitude.yaw);

	anotc_add_short(&frame, (short)(flight.imu.gyro.value.x*100));
	anotc_add_short(&frame, (short)(flight.imu.gyro.value.y*100));
	anotc_add_short(&frame, (short)(flight.imu.gyro.value.z*100));

	anotc_add_ushort(&frame, (unsigned short)(flight.setpoints.throttle*1000));
	anotc_add_short(&frame, (short)(flight.setpoints.roll*100));
	anotc_add_short(&frame, (short)(flight.setpoints.pitch*100));
	anotc_add_short(&frame, (short)(flight.setpoints.yaw*100));

	anotc_add_ushort(&frame, flight.mixer.motor[0]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[1]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[2]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[3]->value);

	anotc_add_short(&frame, (short)(angle_rate_pid[PID_ROLL].P*100));
	anotc_add_short(&frame, (short)(angle_rate_pid[PID_ROLL].I*100));
	anotc_add_short(&frame, (short)(angle_rate_pid[PID_ROLL].D*100));
	anotc_add_short(&frame, (short)(angle_rate_pid[PID_ROLL].output*100));

	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_PITCH].P*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_PITCH].I*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_PITCH].D*100));
	// frame.data[frame.len++] = (char)(angle_rate_pid[PID_PITCH].output*100);

	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_YAW].P*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_YAW].I*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_YAW].D*100));
	// frame.data[frame.len++] = (char)(angle_rate_pid[PID_YAW].output*100);

	// anotc_add_short(&frame, (short)(angle_pid[PID_ROLL].P*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_ROLL].I*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_ROLL].D*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_ROLL].output*100));

	// anotc_add_short(&frame, (short)(angle_pid[PID_PITCH].P*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_PITCH].I*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_PITCH].D*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_PITCH].output*100));

	// anotc_add_short(&frame, (short)(angle_pid[PID_YAW].P*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_YAW].I*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_YAW].D*100));
	// anotc_add_short(&frame, (short)(angle_pid[PID_YAW].output*100));

	

	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_ROLL].desired*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_PITCH].desired*100));
	// anotc_add_short(&frame, (short)(angle_rate_pid[PID_YAW].desired*100));

	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char*)(&frame), ANOTC_V8_HEAD_SIZE + frame.len + 2);
}