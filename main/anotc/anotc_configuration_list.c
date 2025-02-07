#include "misc/config.h"

#include "platform/drivers/wifi.h"
#include "flight/flight.h"

enum ANOTC_CONFIG_INFO_PAR_ID {
	ANOTC_CONFIG_PAR_WIFI_NAME,
	ANOTC_CONFIG_PAR_WIFI_PASSWORD,
	ANOTC_CONFIG_PAR_ACCEL_K_X,
	ANOTC_CONFIG_PAR_ACCEL_K_Y,
	ANOTC_CONFIG_PAR_ACCEL_K_Z,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_X,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_Y,
	ANOTC_CONFIG_PAR_ACCEL_OFFSET_Z,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_X,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_Y,
	ANOTC_CONFIG_PAR_GYRO_OFFSET_Z
};

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
static struct anotc_config_info configuration_list[] = {
	{
		.par_id=ANOTC_CONFIG_PAR_WIFI_NAME,
		.type=ANOTC_PAR_TYPE_STRING,
		.par_name="wifi_name",
		.par_info="",
		.get = (void*)wifi_get_name,
		.set = set_wifi_name
	},
	{
		.par_id=ANOTC_CONFIG_PAR_WIFI_PASSWORD,
		.type=ANOTC_PAR_TYPE_STRING,
		.par_name="wifi_password",
		.par_info="",
		.get = (void*)wifi_get_password,
		.set = set_wifi_pwd
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
	}
};

#define ANOTC_CONFIGURATION_LIST_SIZE sizeof(configuration_list)/sizeof(struct anotc_config_info)