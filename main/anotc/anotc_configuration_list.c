#include "misc/config.h"

#include "platform/drivers/wifi.h"
#include "flight/flight.h"
#include "flight/control.h"

enum ANOTC_CONFIG_INFO_PAR_ID {
	ANOTC_CONFIG_PAR_WIFI_NAME,
	ANOTC_CONFIG_PAR_WIFI_PASSWORD,
	ANOTC_CONFIG_PAR_WIFI_UDP_PORT,
	ANOTC_CONFIG_PAR_ACCEL_K_X,
	ANOTC_CONFIG_PAR_ACCEL_K_Y,
	ANOTC_CONFIG_PAR_ACCEL_K_Z,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_X,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_Y,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_Z,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_X,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_Y,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_Z,
	ANOTC_CONFIG_PAR_MAG_K_X,
	ANOTC_CONFIG_PAR_MAG_K_Y,
	ANOTC_CONFIG_PAR_MAG_K_Z,
	ANOTC_CONFIG_PAR_MAG_OFFSET_X,
	ANOTC_CONFIG_PAR_MAG_OFFSET_Y,
	ANOTC_CONFIG_PAR_MAG_OFFSET_Z,
	ANOTC_CONFIG_PAR_ESC_PROTOCOL,
	ANOTC_CONFIG_PAR_MOTOR_MAPPING,
	ANOTC_CONFIG_PAR_PID_ROLL_P,
	ANOTC_CONFIG_PAR_PID_ROLL_I,
	ANOTC_CONFIG_PAR_PID_ROLL_D,
	ANOTC_CONFIG_PAR_PID_ROLL_D_F,
	ANOTC_CONFIG_PAR_PID_PITCH_P,
	ANOTC_CONFIG_PAR_PID_PITCH_I,
	ANOTC_CONFIG_PAR_PID_PITCH_D,
	ANOTC_CONFIG_PAR_PID_PITCH_D_F,
	ANOTC_CONFIG_PAR_PID_YAW_P,
	ANOTC_CONFIG_PAR_PID_YAW_I,
	ANOTC_CONFIG_PAR_PID_YAW_D,
	ANOTC_CONFIG_PAR_PID_YAW_D_F,
	ANOTC_CONFIG_PAR_PID_ROLL_RATE_P,
	ANOTC_CONFIG_PAR_PID_ROLL_RATE_I,
	ANOTC_CONFIG_PAR_PID_ROLL_RATE_D,
	ANOTC_CONFIG_PAR_PID_ROLL_RATE_D_F,
	ANOTC_CONFIG_PAR_PID_PITCH_RATE_P,
	ANOTC_CONFIG_PAR_PID_PITCH_RATE_I,
	ANOTC_CONFIG_PAR_PID_PITCH_RATE_D,
	ANOTC_CONFIG_PAR_PID_PITCH_RATE_D_F,
	ANOTC_CONFIG_PAR_PID_YAW_RATE_P,
	ANOTC_CONFIG_PAR_PID_YAW_RATE_I,
	ANOTC_CONFIG_PAR_PID_YAW_RATE_D,
	ANOTC_CONFIG_PAR_PID_YAW_RATE_D_F,
	ANOTC_CONFIG_PAR_RC_PROTOCOL

};

static unsigned int tmp_get_value;

char* set_wifi_name(void *value)
{
	if (strlen(value)>32) {
		return "Wifi name must less than 32 characters";
	}
	wifi_set_name(value);
	return 0;
}

char* set_wifi_pwd(void *value)
{
	if (strlen(value)>64) {
		return "Wifi pwd must less than 32 characters";
	}
	wifi_set_password(value);
	return 0;
}

char* set_wifi_udp_port(void *value)
{
	wifi_set_udp_port(*((unsigned short*)value));
	return 0;
}

void* get_accel_calibration_x_k()
{
	return (void*)&(flight.imu.accel.calibration.x_k);
}
char* set_accel_calibration_x_k(void *value)
{
	flight.imu.accel.calibration.x_k = *((float*)value);
	config_write_float("accel_k.x", flight.imu.accel.calibration.x_k);
	return 0;
}

void* get_accel_calibration_y_k()
{
	return (void*)&(flight.imu.accel.calibration.y_k);
}
char* set_accel_calibration_y_k(void *value)
{
	flight.imu.accel.calibration.y_k = *((float*)value);
	config_write_float("accel_k.y", flight.imu.accel.calibration.y_k);
	return 0;
}

void* get_accel_calibration_z_k()
{
	return (void*)&(flight.imu.accel.calibration.z_k);
}
char* set_accel_calibration_z_k(void *value)
{
	flight.imu.accel.calibration.z_k = *((float*)value);
	config_write_float("accel_k.z", flight.imu.accel.calibration.z_k);
	return 0;
}

void* get_accel_calibration_x_offset()
{
	return (void*)&(flight.imu.accel.calibration.x_offset);
}
char* set_accel_calibration_x_offset(void *value)
{
	flight.imu.accel.calibration.x_offset = *((float*)value);
	config_write_float("accel_offset.x", flight.imu.accel.calibration.x_offset);
	return 0;
}

void* get_accel_calibration_y_offset()
{
	return (void*)&(flight.imu.accel.calibration.y_offset);
}
char* set_accel_calibration_y_offset(void *value)
{
	flight.imu.accel.calibration.y_offset = *((float*)value);
	config_write_float("accel_offset.y", flight.imu.accel.calibration.y_offset);
	return 0;
}

void* get_accel_calibration_z_offset()
{
	return (void*)&(flight.imu.accel.calibration.z_offset);
}
char* set_accel_calibration_z_offset(void *value)
{
	flight.imu.accel.calibration.z_offset = *((float*)value);
	config_write_float("accel_offset.z", flight.imu.accel.calibration.z_offset);
	return 0;
}

void* get_gyro_calibration_x_offset()
{
	return (void*)&(flight.imu.gyro.calibration.x_offset);
}
char* set_gyro_calibration_x_offset(void *value)
{
	flight.imu.gyro.calibration.x_offset = *((float*)value);
	config_write_float("gyro_offset.x", flight.imu.gyro.calibration.x_offset);
	return 0;
}

void* get_gyro_calibration_y_offset()
{
	return (void*)&(flight.imu.gyro.calibration.y_offset);
}
char* set_gyro_calibration_y_offset(void *value)
{
	flight.imu.gyro.calibration.y_offset = *((float*)value);
	config_write_float("gyro_offset.y", flight.imu.gyro.calibration.y_offset);
	return 0;
}

void* get_gyro_calibration_z_offset()
{
	return (void*)&(flight.imu.gyro.calibration.z_offset);
}
char* set_gyro_calibration_z_offset(void *value)
{
	flight.imu.gyro.calibration.z_offset = *((float*)value);
	config_write_float("gyro_offset.z", flight.imu.gyro.calibration.z_offset);
	return 0;
}

void* get_mag_calibration_x_k()
{
	return (void*)&(flight.compass.calibration.x_k);
}
char* set_mag_calibration_x_k(void *value)
{
	flight.compass.calibration.x_k = *((float*)value);
	config_write_float("mag_k.x", flight.compass.calibration.x_k);
	return 0;
}

void* get_mag_calibration_y_k()
{
	return (void*)&(flight.compass.calibration.y_k);
}
char* set_mag_calibration_y_k(void *value)
{
	flight.compass.calibration.y_k = *((float*)value);
	config_write_float("mag_k.y", flight.compass.calibration.y_k);
	return 0;
}

void* get_mag_calibration_z_k()
{
	return (void*)&(flight.compass.calibration.z_k);
}
char* set_mag_calibration_z_k(void *value)
{
	flight.compass.calibration.z_k = *((float*)value);
	config_write_float("mag_k.z", flight.compass.calibration.z_k);
	return 0;
}

void* get_mag_calibration_x_offset()
{
	return (void*)&(flight.compass.calibration.x_offset);
}
char* set_mag_calibration_x_offset(void *value)
{
	flight.compass.calibration.x_offset = *((float*)value);
	config_write_float("mag_offset.x", flight.compass.calibration.x_offset);
	return 0;
}

void* get_mag_calibration_y_offset()
{
	return (void*)&(flight.compass.calibration.y_offset);
}
char* set_mag_calibration_y_offset(void *value)
{
	flight.compass.calibration.y_offset = *((float*)value);
	config_write_float("mag_offset.y", flight.compass.calibration.y_offset);
	return 0;
}

void* get_mag_calibration_z_offset()
{
	return (void*)&(flight.compass.calibration.z_offset);
}
char* set_mag_calibration_z_offset(void *value)
{
	flight.compass.calibration.z_offset = *((float*)value);
	config_write_float("mag_offset.z", flight.compass.calibration.z_offset);
	return 0;
}

void* get_esc_protocol()
{
	unsigned char value=0;
	config_read_uchar("esc_protocol", &value);
	tmp_get_value = value;
	return &tmp_get_value;
}
char* set_esc_protocol(void *value)
{
	if (flight.status!=FLIGHT_STATUS_READY) {
		return "Flight isn't in ready status";
	}
	config_write_uchar("esc_protocol", *((unsigned char*)value));
	return 0;
}

void* get_motor_mapping()
{
	unsigned long int motor_mapping=0x3210;
	config_read_uint("motor_mapping", &motor_mapping);
	tmp_get_value = motor_mapping;
	return &tmp_get_value;
}
char* set_motor_mapping(void *value)
{
	if (flight.status!=FLIGHT_STATUS_READY) {
		return "Flight isn't in ready status";
	}
	unsigned long int motor_mapping = *((unsigned long int*)value);
	config_write_uint("motor_mapping", motor_mapping);
	for (int i=0;i<sizeof(__motors)/sizeof(__motors[0]);i++) {
		flight.mixer.motor[i] = &__motors[MOTOR_GET_INDEX(MOTOR_MAPPING_VALUE(motor_mapping, i))];
	}
	return 0;
}

void* get_pid_roll_p()
{
	return (void*)(&angle_pid[PID_ROLL].kp);
}
char* set_pid_roll_p(void *value)
{
	angle_pid[PID_ROLL].kp = *((float*)value);
	config_write_float("rol_pid.p", angle_pid[PID_ROLL].kp);
	return 0;
}

void* get_pid_roll_i()
{
	return (void*)(&angle_pid[PID_ROLL].ki);
}
char* set_pid_roll_i(void *value)
{
	angle_pid[PID_ROLL].ki = *((float*)value);
	config_write_float("rol_pid.i", angle_pid[PID_ROLL].ki);
	return 0;
}

void* get_pid_roll_d()
{
	return (void*)(&angle_pid[PID_ROLL].kd);
}
char* set_pid_roll_d(void *value)
{
	angle_pid[PID_ROLL].kd = *((float*)value);
	config_write_float("rol_pid.d", angle_pid[PID_ROLL].kd);
	return 0;
}

void* get_pid_roll_d_f()
{
	return (void*)(&angle_pid[PID_ROLL].filter.cut_off_freq);
}
char* set_pid_roll_d_f(void *value)
{
	angle_pid[PID_ROLL].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("rol_pid.d_f", angle_pid[PID_ROLL].filter.cut_off_freq);
	return 0;
}

void* get_pid_pitch_p()
{
	return (void*)&(angle_pid[PID_PITCH].kp);
}
char* set_pid_pitch_p(void *value)
{
	angle_pid[PID_PITCH].kp = *((float*)value);
	config_write_float("pit_pid.p", angle_pid[PID_PITCH].kp);
	return 0;
}

void* get_pid_pitch_i()
{
	return (void*)&angle_pid[PID_PITCH].ki;
}
char* set_pid_pitch_i(void *value)
{
	angle_pid[PID_PITCH].ki = *((float*)value);
	config_write_float("pit_pid.i", angle_pid[PID_PITCH].ki);
	return 0;
}

void* get_pid_pitch_d()
{
	return (void*)&angle_pid[PID_PITCH].kd;
}
char* set_pid_pitch_d(void *value)
{
	angle_pid[PID_PITCH].kd = *((float*)value);
	config_write_float("pit_pid.d", angle_pid[PID_PITCH].kd);
	return 0;
}

void* get_pid_pitch_d_f()
{
	return (void*)(&angle_pid[PID_PITCH].filter.cut_off_freq);
}
char* set_pid_pitch_d_f(void *value)
{
	angle_pid[PID_PITCH].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("pit_pid.d_f", angle_pid[PID_PITCH].filter.cut_off_freq);
	return 0;
}

void* get_pid_yaw_p()
{
	return (void*)&angle_pid[PID_YAW].kp;
}
char* set_pid_yaw_p(void *value)
{
	angle_pid[PID_YAW].kp = *((float*)value);
	config_write_float("yaw_pid.p", angle_pid[PID_YAW].kp);
	return 0;
}

void* get_pid_yaw_i()
{
	return (void*)&angle_pid[PID_YAW].ki;
}
char* set_pid_yaw_i(void *value)
{
	angle_pid[PID_YAW].ki = *((float*)value);
	config_write_float("yaw_pid.i", angle_pid[PID_YAW].ki);
	return 0;
}

void* get_pid_yaw_d()
{
	return (void*)&angle_pid[PID_YAW].kd;
}
char* set_pid_yaw_d(void *value)
{
	angle_pid[PID_YAW].kd = *((float*)value);
	config_write_float("yaw_pid.d", angle_pid[PID_YAW].kd);
	return 0;
}

void* get_pid_yaw_d_f()
{
	return (void*)(&angle_pid[PID_YAW].filter.cut_off_freq);
}
char* set_pid_yaw_d_f(void *value)
{
	angle_pid[PID_YAW].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("yaw_pid.d_f", angle_pid[PID_YAW].filter.cut_off_freq);
	return 0;
}

void* get_pid_roll_rate_p()
{
	return (void*)&angle_rate_pid[PID_ROLL].kp;
}
char* set_pid_roll_rate_p(void *value)
{
	angle_rate_pid[PID_ROLL].kp = *((float*)value);
	config_write_float("rol_r_pid.p", angle_rate_pid[PID_ROLL].kp);
	return 0;
}

void* get_pid_roll_rate_i()
{
	return (void*)&angle_rate_pid[PID_ROLL].ki;
}
char* set_pid_roll_rate_i(void *value)
{
	angle_rate_pid[PID_ROLL].ki = *((float*)value);
	config_write_float("rol_r_pid.i", angle_rate_pid[PID_ROLL].ki);
	return 0;
}

void* get_pid_roll_rate_d()
{
	return (void*)&angle_rate_pid[PID_ROLL].kd;
}
char* set_pid_roll_rate_d(void *value)
{
	angle_rate_pid[PID_ROLL].kd = *((float*)value);
	config_write_float("rol_r_pid.d", angle_rate_pid[PID_ROLL].kd);
	return 0;
}

void* get_pid_roll_rate_d_f()
{
	return (void*)(&angle_rate_pid[PID_ROLL].filter.cut_off_freq);
}
char* set_pid_roll_rate_d_f(void *value)
{
	angle_rate_pid[PID_ROLL].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("rol_r_pid.d_f", angle_rate_pid[PID_ROLL].filter.cut_off_freq);
	return 0;
}

void* get_pid_pitch_rate_p()
{
	return (void*)&angle_rate_pid[PID_PITCH].kp;
}
char* set_pid_pitch_rate_p(void *value)
{
	angle_rate_pid[PID_PITCH].kp = *((float*)value);
	config_write_float("pit_r_pid.p", angle_rate_pid[PID_PITCH].kp);
	return 0;
}

void* get_pid_pitch_rate_i()
{
	return (void*)&angle_rate_pid[PID_PITCH].ki;
}
char* set_pid_pitch_rate_i(void *value)
{
	angle_rate_pid[PID_PITCH].ki = *((float*)value);
	config_write_float("pit_r_pid.i", angle_rate_pid[PID_PITCH].ki);
	return 0;
}

void* get_pid_pitch_rate_d()
{
	return (void*)&angle_rate_pid[PID_PITCH].kd;
}
char* set_pid_pitch_rate_d(void *value)
{
	angle_rate_pid[PID_PITCH].kd = *((float*)value);
	config_write_float("pit_r_pid.d", angle_rate_pid[PID_PITCH].kd);
	return 0;
}

void* get_pid_pitch_rate_d_f()
{
	return (void*)(&angle_rate_pid[PID_PITCH].filter.cut_off_freq);
}
char* set_pid_pitch_rate_d_f(void *value)
{
	angle_rate_pid[PID_PITCH].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("pit_r_pid.d_f", angle_rate_pid[PID_PITCH].filter.cut_off_freq);
	return 0;
}

void* get_pid_yaw_rate_p()
{
	return (void*)&angle_rate_pid[PID_YAW].kp;
}
char* set_pid_yaw_rate_p(void *value)
{
	angle_rate_pid[PID_YAW].kp = *((float*)value);
	config_write_float("yaw_r_pid.p", angle_rate_pid[PID_YAW].kp);
	return 0;
}

void* get_pid_yaw_rate_i()
{
	return (void*)&angle_rate_pid[PID_YAW].ki;
}
char* set_pid_yaw_rate_i(void *value)
{
	angle_rate_pid[PID_YAW].ki = *((float*)value);
	config_write_float("yaw_r_pid.i", angle_rate_pid[PID_YAW].ki);
	return 0;
}

void* get_pid_yaw_rate_d()
{
	return (void*)&angle_rate_pid[PID_YAW].kd;
}
char* set_pid_yaw_rate_d(void *value)
{
	angle_rate_pid[PID_YAW].kd = *((float*)value);
	config_write_float("yaw_r_pid.d", angle_rate_pid[PID_YAW].kd);
	return 0;
}

void* get_pid_yaw_rate_d_f()
{
	return (void*)(&angle_rate_pid[PID_YAW].filter.cut_off_freq);
}
char* set_pid_yaw_rate_d_f(void *value)
{
	angle_rate_pid[PID_YAW].filter.cut_off_freq = *((unsigned short*)value);
	config_write_ushort("yaw_r_pid.d_f", angle_rate_pid[PID_YAW].filter.cut_off_freq);
	return 0;
}

void* get_rc_protocol()
{
	config_read_uchar("rc_protocol", (unsigned char*)&tmp_get_value);
	return &tmp_get_value;
}
char* set_rc_protocol(void *value)
{
	if (flight.status!=FLIGHT_STATUS_READY) {
		return "Flight isn't in ready status";
	}
	unsigned char protocol = *((unsigned char*)value);
	config_write_uchar("rc_protocol", protocol);
	return 0;
}

static struct anotc_config_info configuration_list[] = {
	{
		.par_id=ANOTC_CONFIG_PAR_WIFI_NAME,
		.type=ANOTC_PAR_TYPE_STRING,
		.par_name="wifi.name",
		.par_info="",
		.get = (void*)wifi_get_name,
		.set = set_wifi_name
	},
	{
		.par_id=ANOTC_CONFIG_PAR_WIFI_PASSWORD,
		.type=ANOTC_PAR_TYPE_STRING,
		.par_name="wifi.password",
		.par_info="",
		.get = (void*)wifi_get_password,
		.set = set_wifi_pwd
	},
	{
		.par_id=ANOTC_CONFIG_PAR_WIFI_UDP_PORT,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="wifi.udp.port",
		.par_info="",
		.get = (void*)wifi_get_udp_port,
		.set = set_wifi_udp_port
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_K_X,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_k.x",
		.par_info="",
		.get = get_accel_calibration_x_k,
		.set = set_accel_calibration_x_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_K_Y,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_k.y",
		.par_info="",
		.get = get_accel_calibration_y_k,
		.set = set_accel_calibration_y_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_K_Z,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_k.z",
		.par_info="",
		.get = get_accel_calibration_z_k,
		.set = set_accel_calibration_z_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_OFFSET_X,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_offset.x",
		.par_info="",
		.get = get_accel_calibration_x_offset,
		.set = set_accel_calibration_x_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_OFFSET_Y,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_offset.y",
		.par_info="",
		.get = get_accel_calibration_y_offset,
		.set = set_accel_calibration_y_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ACCEL_OFFSET_Z,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="accel_offset.z",
		.par_info="",
		.get = get_accel_calibration_z_offset,
		.set = set_accel_calibration_z_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_GYRO_OFFSET_X,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="gyro_offset.x",
		.par_info="",
		.get = get_gyro_calibration_x_offset,
		.set = set_gyro_calibration_x_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_GYRO_OFFSET_Y,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="gyro_offset.y",
		.par_info="",
		.get = get_gyro_calibration_y_offset,
		.set = set_gyro_calibration_y_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_GYRO_OFFSET_Z,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="gyro_offset.z",
		.par_info="",
		.get = get_gyro_calibration_z_offset,
		.set = set_gyro_calibration_z_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_K_X,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_k.x",
		.par_info="",
		.get = get_mag_calibration_x_k,
		.set = set_mag_calibration_x_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_K_Y,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_k.y",
		.par_info="",
		.get = get_mag_calibration_y_k,
		.set = set_mag_calibration_y_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_K_Z,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_k.z",
		.par_info="",
		.get = get_mag_calibration_z_k,
		.set = set_mag_calibration_z_k
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_OFFSET_X,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_offset.x",
		.par_info="",
		.get = get_mag_calibration_x_offset,
		.set = set_mag_calibration_x_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_OFFSET_Y,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_offset.y",
		.par_info="",
		.get = get_mag_calibration_y_offset,
		.set = set_mag_calibration_y_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MAG_OFFSET_Z,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="mag_offset.z",
		.par_info="",
		.get = get_mag_calibration_z_offset,
		.set = set_mag_calibration_z_offset
	},
	{
		.par_id=ANOTC_CONFIG_PAR_ESC_PROTOCOL,
		.type=ANOTC_PAR_TYPE_UINT8,
		.par_name="esc_protocol",
		.par_info="0.Bushed 1.DSHOT300",
		.get = get_esc_protocol,
		.set = set_esc_protocol
	},
	{
		.par_id=ANOTC_CONFIG_PAR_MOTOR_MAPPING,
		.type=ANOTC_PAR_TYPE_UINT32,
		.par_name="motor_mapping",
		.par_info="bit 4:reverse",
		.get = get_motor_mapping,
		.set = set_motor_mapping
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll.p",
		.par_info="",
		.get = get_pid_roll_p,
		.set = set_pid_roll_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll.i",
		.par_info="",
		.get = get_pid_roll_i,
		.set = set_pid_roll_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll.d",
		.par_info="",
		.get = get_pid_roll_d,
		.set = set_pid_roll_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.roll.d_f",
		.par_info="",
		.get = get_pid_roll_d_f,
		.set = set_pid_roll_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch.p",
		.par_info="",
		.get = get_pid_pitch_p,
		.set = set_pid_pitch_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch.i",
		.par_info="",
		.get = get_pid_pitch_i,
		.set = set_pid_pitch_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch.d",
		.par_info="",
		.get = get_pid_pitch_d,
		.set = set_pid_pitch_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.pitch.d_f",
		.par_info="",
		.get = get_pid_pitch_d_f,
		.set = set_pid_pitch_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw.p",
		.par_info="",
		.get = get_pid_yaw_p,
		.set = set_pid_yaw_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw.i",
		.par_info="",
		.get = get_pid_yaw_i,
		.set = set_pid_yaw_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw.d",
		.par_info="",
		.get = get_pid_yaw_d,
		.set = set_pid_yaw_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.yaw.d_f",
		.par_info="",
		.get = get_pid_yaw_d_f,
		.set = set_pid_yaw_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_RATE_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll_rate.p",
		.par_info="",
		.get = get_pid_roll_rate_p,
		.set = set_pid_roll_rate_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_RATE_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll_rate.i",
		.par_info="",
		.get = get_pid_roll_rate_i,
		.set = set_pid_roll_rate_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_RATE_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.roll_rate.d",
		.par_info="",
		.get = get_pid_roll_rate_d,
		.set = set_pid_roll_rate_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_ROLL_RATE_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.roll_rate.d_f",
		.par_info="",
		.get = get_pid_roll_rate_d_f,
		.set = set_pid_roll_rate_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_RATE_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch_rate.p",
		.par_info="",
		.get = get_pid_pitch_rate_p,
		.set = set_pid_pitch_rate_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_RATE_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch_rate.i",
		.par_info="",
		.get = get_pid_pitch_rate_i,
		.set = set_pid_pitch_rate_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_RATE_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.pitch_rate.d",
		.par_info="",
		.get = get_pid_pitch_rate_d,
		.set = set_pid_pitch_rate_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_PITCH_RATE_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.pitch_rate.d_f",
		.par_info="",
		.get = get_pid_pitch_rate_d_f,
		.set = set_pid_pitch_rate_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_RATE_P,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw_rate.p",
		.par_info="",
		.get = get_pid_yaw_rate_p,
		.set = set_pid_yaw_rate_p
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_RATE_I,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw_rate.i",
		.par_info="",
		.get = get_pid_yaw_rate_i,
		.set = set_pid_yaw_rate_i
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_RATE_D,
		.type=ANOTC_PAR_TYPE_FLOAT,
		.par_name="pid.yaw_rate.d",
		.par_info="",
		.get = get_pid_yaw_rate_d,
		.set = set_pid_yaw_rate_d
	},
	{
		.par_id=ANOTC_CONFIG_PAR_PID_YAW_RATE_D_F,
		.type=ANOTC_PAR_TYPE_UINT16,
		.par_name="pid.yaw_rate.d_f",
		.par_info="",
		.get = get_pid_yaw_rate_d_f,
		.set = set_pid_yaw_rate_d_f
	},
	{
		.par_id=ANOTC_CONFIG_PAR_RC_PROTOCOL,
		.type=ANOTC_PAR_TYPE_UINT8,
		.par_name="rc_protocol",
		.par_info="0.ibus 1.pc",
		.get = get_rc_protocol,
		.set = set_rc_protocol
	}
};

#define ANOTC_CONFIGURATION_LIST_SIZE sizeof(configuration_list)/sizeof(struct anotc_config_info)