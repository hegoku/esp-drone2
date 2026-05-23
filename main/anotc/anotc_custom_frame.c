#include "anotc/anotc.h"
#include "anotc/anotc_custom_frame.h"
#include "flight/flight.h"
#include "misc/util.h"
#include "flight/control.h"
#include "sdkconfig.h"

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

	anotc_add_float(&frame, flight.attitude.roll);
	anotc_add_float(&frame, flight.attitude.pitch);
	anotc_add_float(&frame, flight.attitude.yaw);

	anotc_add_float(&frame, flight.imu.gyro.value.x);
	anotc_add_float(&frame, flight.imu.gyro.value.y);
	anotc_add_float(&frame, flight.imu.gyro.value.z);

	anotc_add_ushort(&frame, (unsigned short)(flight.setpoints.throttle*1000));
	anotc_add_float(&frame, flight.setpoints.roll);
	anotc_add_float(&frame, flight.setpoints.pitch);
	anotc_add_float(&frame, flight.setpoints.yaw);

	anotc_add_ushort(&frame, flight.mixer.motor[0]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[1]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[2]->value);
	anotc_add_ushort(&frame, flight.mixer.motor[3]->value);

	anotc_add_short(&frame, flight.imu.accel.raw.x);
	anotc_add_short(&frame, flight.imu.accel.raw.y);
	anotc_add_short(&frame, flight.imu.accel.raw.z);

	anotc_add_short(&frame, flight.imu.gyro.raw.x);
	anotc_add_short(&frame, flight.imu.gyro.raw.y);
	anotc_add_short(&frame, flight.imu.gyro.raw.z);

	anotc_add_float(&frame, angle_rate_pid[PID_ROLL].desired);
	anotc_add_float(&frame, angle_rate_pid[PID_PITCH].desired);
	anotc_add_float(&frame, angle_rate_pid[PID_YAW].desired);

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

#ifdef CONFIG_ANGLE_PID_ALGORITHM_EULER_PID
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
#elif defined(CONFIG_ANGLE_PID_ALGORITHM_Q_PID)
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, angle_q_pid.output_roll);

	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, angle_q_pid.output_pitch);

	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, 0);
	anotc_add_float(&frame, angle_q_pid.output_yaw);
#endif


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