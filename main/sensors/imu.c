#include <stdio.h>
#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/iir_filter.h"
#include "clocksource/imu_source.h"
#include "misc/config.h"

static struct iir_filter_param acc_iir[3];
static struct iir_filter_param gyr_iir[3];

void init_imu(struct imu_sensor *sensor)
{
	if (sensor->status != IMU_STATUS_ON)
		return;
	
	sensor->accel.calibration.x_k = 1.0;
	sensor->accel.calibration.y_k = 1.0;
	sensor->accel.calibration.z_k = 1.0;

	for (int i = 0; i < 3; i++)
	{
		acc_iir[i].cut_off_freq = 10;
		acc_iir[i].freq = sensor->freq;
		iir_filter_init(&acc_iir[i]);

		gyr_iir[i].cut_off_freq = 15;
		gyr_iir[i].freq = sensor->freq;
		iir_filter_init(&gyr_iir[i]);
	}

	config_read_float("accel_k.x", &(sensor->accel.calibration.x_k));
	config_read_float("accel_k.y", &(sensor->accel.calibration.y_k));
	config_read_float("accel_k.z", &(sensor->accel.calibration.z_k));
	
	config_read_float("accel_offset.x", &(sensor->accel.calibration.x_offset));
	config_read_float("accel_offset.y", &(sensor->accel.calibration.y_offset));
	config_read_float("accel_offset.z", &(sensor->accel.calibration.z_offset));

	config_read_float("gyro_offset.x", &(sensor->gyro.calibration.x_offset));
	config_read_float("gyro_offset.y", &(sensor->gyro.calibration.y_offset));
	config_read_float("gyro_offset.z", &(sensor->gyro.calibration.z_offset));

	// sys_timer_set(imu_timer);
}

void imu_filter(struct imu_sensor *sensor)
{
	sensor->accel.unfiltered.x = sensor->accel.calibration.x_k * sensor->accel.unfiltered.x - sensor->accel.calibration.x_offset;
	sensor->accel.unfiltered.y = sensor->accel.calibration.y_k * sensor->accel.unfiltered.y - sensor->accel.calibration.y_offset;
	sensor->accel.unfiltered.z = sensor->accel.calibration.z_k * sensor->accel.unfiltered.z - sensor->accel.calibration.z_offset;
	sensor->accel.value.x = iir_filter(&acc_iir[0], sensor->accel.unfiltered.x);
	sensor->accel.value.y = iir_filter(&acc_iir[1], sensor->accel.unfiltered.y);
	sensor->accel.value.z = iir_filter(&acc_iir[2], sensor->accel.unfiltered.z);

	sensor->gyro.unfiltered.x -=  sensor->gyro.calibration.x_offset;
	sensor->gyro.unfiltered.y -=  sensor->gyro.calibration.y_offset;
	sensor->gyro.unfiltered.z -=  sensor->gyro.calibration.z_offset;
	sensor->gyro.value.x = iir_filter(&gyr_iir[0], sensor->gyro.unfiltered.x);
	sensor->gyro.value.y = iir_filter(&gyr_iir[1], sensor->gyro.unfiltered.y);
	sensor->gyro.value.z = iir_filter(&gyr_iir[2], sensor->gyro.unfiltered.z);
}