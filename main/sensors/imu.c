#include <stdio.h>
#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/iir_filter.h"
#include "clocksource/imu_source.h"

static struct iir_filter_param acc_iir[3];
static struct iir_filter_param gyr_iir[3];

void init_imu(struct imu_sensor *sensor)
{
	if (sensor->status != IMU_STATUS_ON)
		return;

	for (int i = 0; i < 3; i++)
	{
		acc_iir[i].cut_off_freq = 10;
		acc_iir[i].freq = sensor->freq;
		iir_filter_init(&acc_iir[i]);

		gyr_iir[i].cut_off_freq = 15;
		gyr_iir[i].freq = sensor->freq;
		iir_filter_init(&gyr_iir[i]);
	}

	// sys_timer_set(imu_timer);
}

void imu_filter(struct imu_sensor *sensor)
{
	sensor->accel.value.x = iir_filter(&acc_iir[0], sensor->accel.unfiltered.x);
	sensor->accel.value.y = iir_filter(&acc_iir[1], sensor->accel.unfiltered.y);
	sensor->accel.value.z = iir_filter(&acc_iir[2], sensor->accel.unfiltered.z);

	sensor->gyro.value.x = iir_filter(&gyr_iir[0], sensor->gyro.unfiltered.x);
	sensor->gyro.value.y = iir_filter(&gyr_iir[1], sensor->gyro.unfiltered.y);
	sensor->gyro.value.z = iir_filter(&gyr_iir[2], sensor->gyro.unfiltered.z);
}