#include "sensors/compass.h"
#include "misc/iir_filter.h"
#include "misc/config.h"

static struct iir_filter_param compass_iir[3];

void init_compass(struct compass_sensor *sensor)
{
	sensor->calibration.x_k = 1.0;
	sensor->calibration.y_k = 1.0;
	sensor->calibration.z_k = 1.0;

	config_read_float("mag_k.x", &(sensor->calibration.x_k));
	config_read_float("mag_k.y", &(sensor->calibration.y_k));
	config_read_float("mag_k.z", &(sensor->calibration.z_k));
	config_read_float("mag_offset.x", &(sensor->calibration.x_offset));
	config_read_float("mag_offset.y", &(sensor->calibration.y_offset));
	config_read_float("mag_offset.z", &(sensor->calibration.z_offset));

	if (sensor->status != COMPASS_STATUS_ON)
		return;

	for (int i = 0; i < 3; i++)
	{
		compass_iir[i].cut_off_freq = 10;
		compass_iir[i].freq = sensor->freq;
		iir_filter_init(&compass_iir[i]);
	}
}

void compass_filter(struct compass_sensor *sensor)
{
	sensor->value.x = sensor->calibration.x_k * (sensor->value.x - sensor->calibration.x_offset);
	sensor->value.y = sensor->calibration.y_k * (sensor->value.y - sensor->calibration.y_offset);
	sensor->value.z = sensor->calibration.z_k * (sensor->value.z - sensor->calibration.z_offset);
	sensor->value.x = iir_filter(&compass_iir[0], sensor->value.x);
	sensor->value.y = iir_filter(&compass_iir[1], sensor->value.y);
	sensor->value.z = iir_filter(&compass_iir[2], sensor->value.z);
}