#ifndef SENSOR_SENSOR_ROTATION_H
#define SENSOR_SENSOR_ROTATION_H

enum sensor_rotation : unsigned char
{
	ROTATION_NONE                = 0,
	ROTATION_YAW_45              = 1,
	ROTATION_YAW_90              = 2,
	ROTATION_YAW_135             = 3,
	ROTATION_YAW_180             = 4,
	ROTATION_YAW_225             = 5,
	ROTATION_YAW_270             = 6,
	ROTATION_YAW_315             = 7,
	ROTATION_MAX,
};

void sensor_rotate_3f(enum sensor_rotation rot, float *x, float *y, float *z);

#endif