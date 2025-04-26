#include "sensors/compass.h"
#include "misc/iir_filter.h"
#include "misc/config.h"

static struct iir_filter_param compass_iir[3];

void init_compass(struct compass_sensor *sensor)
{
	sensor->calibration.x_k[0] = 0.9643;
	sensor->calibration.x_k[1] = -0.0188;
	sensor->calibration.x_k[2] = -0.0062;
	sensor->calibration.y_k[0] = -0.0188;
	sensor->calibration.y_k[1] = 1.0297;
	sensor->calibration.y_k[2] = 0.0172;
	sensor->calibration.z_k[0] = -0.0062;
	sensor->calibration.z_k[1] = 0.0172;
	sensor->calibration.z_k[2] = 1.0078;

	config_read_float("mag_k.x1", &(sensor->calibration.x_k[0]));
	config_read_float("mag_k.x2", &(sensor->calibration.x_k[1]));
	config_read_float("mag_k.x3", &(sensor->calibration.x_k[2]));
	config_read_float("mag_k.y1", &(sensor->calibration.y_k[0]));
	config_read_float("mag_k.y2", &(sensor->calibration.y_k[1]));
	config_read_float("mag_k.y3", &(sensor->calibration.y_k[2]));
	config_read_float("mag_k.z1", &(sensor->calibration.z_k[0]));
	config_read_float("mag_k.z2", &(sensor->calibration.z_k[1]));
	config_read_float("mag_k.z3", &(sensor->calibration.z_k[2]));
	config_read_float("mag_offset.x", &(sensor->calibration.x_offset));
	config_read_float("mag_offset.y", &(sensor->calibration.y_offset));
	config_read_float("mag_offset.z", &(sensor->calibration.z_offset));

	if (sensor->status != COMPASS_STATUS_ON)
		return;

	for (int i = 0; i < 3; i++)
	{
		compass_iir[i].cut_off_freq = 5;
		compass_iir[i].freq = sensor->freq;
		iir_filter_init(&compass_iir[i]);
	}
}

void compass_filter(struct compass_sensor *sensor)
{
	if (!IS_COMPASS_DTRY(*sensor)) return;
	float hx,hy,hz;
	hx = sensor->value.x - sensor->calibration.x_offset;
	hy = sensor->value.y - sensor->calibration.y_offset;
	hz = sensor->value.z - sensor->calibration.z_offset;
	sensor->value.x = sensor->calibration.x_k[0] * hx + sensor->calibration.x_k[1] * hy + sensor->calibration.x_k[2] * hz;
	sensor->value.y = sensor->calibration.y_k[0] * hx + sensor->calibration.y_k[1] * hy + sensor->calibration.y_k[2] * hz;
	sensor->value.z = sensor->calibration.z_k[0] * hx + sensor->calibration.z_k[1] * hy + sensor->calibration.z_k[2] * hz;
	sensor->value.x = iir_filter(&compass_iir[0], sensor->value.x);
	sensor->value.y = iir_filter(&compass_iir[1], sensor->value.y);
	sensor->value.z = iir_filter(&compass_iir[2], sensor->value.z);
}