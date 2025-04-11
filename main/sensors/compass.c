#include "sensors/compass.h"
#include "misc/iir_filter.h"

static struct iir_filter_param compass_iir[3];

void init_compass(struct compass_sensor *sensor)
{
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
	sensor->value.x = iir_filter(&compass_iir[0], sensor->value.x);
	sensor->value.y = iir_filter(&compass_iir[1], sensor->value.y);
	sensor->value.z = iir_filter(&compass_iir[2], sensor->value.z);
}