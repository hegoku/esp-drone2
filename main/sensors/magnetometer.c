#include "sensors/magnetometer.h"
#include "misc/low_pass_filter_2p.h"
#include "misc/config.h"
#include "sensors/sensor_rotation.h"

static struct low_pass_filter_2p_param magnetometer_iir[3];

void init_magnetometer(struct magnetometer_sensor *sensor)
{
	sensor->rotation = ROTATION_NONE;
	sensor->declination = 0.0f;
	sensor->calibration.x_k[0] = 1.0f;
	sensor->calibration.x_k[1] = 0.0f;
	sensor->calibration.x_k[2] = 0.0f;
	sensor->calibration.y_k[0] = 1.0f;
	sensor->calibration.y_k[1] = 0.0f;
	sensor->calibration.y_k[2] = 0.0f;
	sensor->calibration.z_k[0] = 0.0f;
	sensor->calibration.z_k[1] = 0.0f;
	sensor->calibration.z_k[2] = 1.0f;

	config_read_uchar("mag.rotation", &(sensor->rotation));
	config_read_float("mag.decl", &(sensor->declination));
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

	if (sensor->status != MAGNETOMETER_STATUS_ON)
		return;

	for (int i = 0; i < 3; i++)
	{
		magnetometer_iir[i].cut_off_freq = 5;
		magnetometer_iir[i].freq = sensor->freq;
		low_pass_filter_2p_init(&magnetometer_iir[i]);
	}
}

void magnetometer_filter(struct magnetometer_sensor *sensor)
{
	if (!IS_MAGNETOMETER_DTRY(*sensor)) return;
	float hx,hy,hz;
	hx = sensor->value.x - sensor->calibration.x_offset;
	hy = sensor->value.y - sensor->calibration.y_offset;
	hz = sensor->value.z - sensor->calibration.z_offset;
	sensor->value.x = sensor->calibration.x_k[0] * hx + sensor->calibration.x_k[1] * hy + sensor->calibration.x_k[2] * hz;
	sensor->value.y = sensor->calibration.y_k[0] * hx + sensor->calibration.y_k[1] * hy + sensor->calibration.y_k[2] * hz;
	sensor->value.z = sensor->calibration.z_k[0] * hx + sensor->calibration.z_k[1] * hy + sensor->calibration.z_k[2] * hz;
	sensor->value.x = low_pass_filter_2p(&magnetometer_iir[0], sensor->value.x);
	sensor->value.y = low_pass_filter_2p(&magnetometer_iir[1], sensor->value.y);
	sensor->value.z = low_pass_filter_2p(&magnetometer_iir[2], sensor->value.z);

	sensor_rotate_3f(sensor->rotation, &(sensor->value.x), &(sensor->value.y), &(sensor->value.z));
}