#include <math.h>
#include "sensors/barometer.h"

void init_baro(struct barometer_sensor *sensor)
{
	sensor->ground_altitude = 0;
	sensor->calibration_count = 0;
}

void baro_pressure2altitude(struct barometer_sensor *sensor)
{
	if (IS_BARO_DTRY(*sensor)) {
		sensor->actual_altitude = 44330.0 * (1.0-pow(sensor->pressure.value/100.0/1013.25, 1.0f/5.255f)) * 100.0;
	}
}

void baro_calcuate_altitude(struct barometer_sensor *sensor)
{
	if (IS_BARO_CALIBRATED(*sensor)) {
		sensor->altitude = sensor->actual_altitude - sensor->ground_altitude;
		return;
	}
	if (IS_BARO_DTRY(*sensor)) {
		sensor->ground_altitude += sensor->actual_altitude;
		sensor->calibration_count++;
		if (sensor->calibration_count>200) {
			sensor->ground_altitude /= (float)sensor->calibration_count;
			sensor->status |= BARO_STATUS_CALIBRATED;
			sensor->calibration_count = 0;
		}
	}
}