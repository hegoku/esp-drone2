#include <math.h>
#include "math/math.h"
#include "sensors/sensor_rotation.h"

struct rot_lookup_t {
	unsigned short roll;
	unsigned short pitch;
	unsigned short yaw;
};

static struct rot_lookup_t rot_lookup[ROTATION_MAX] = {
	{  0,   0,   0 },
	{  0,   0,  45 },
	{  0,   0,  90 },
	{  0,   0, 135 },
	{  0,   0, 180 },
	{  0,   0, 225 },
	{  0,   0, 270 },
	{  0,   0, 315 },
};

struct sensor_rot_matrix {
    float m[3][3];
};

inline static int sensor_rotate_direct(enum sensor_rotation rot, float *x, float *y, float *z)
{
	switch (rot) {
	case ROTATION_YAW_90: {
			float tmp = *x;
			*x = -*y;
			*y = tmp;
		}
		return 1;

	case ROTATION_YAW_180: {
			*x = -*x;
			*y = -*y;
		}
		return 1;

	case ROTATION_YAW_270: {
			float tmp = *x;
			*x = *y;
			*y =-tmp;
		}
		return 1;
	default:
		break;
	}
	return 0;
}

void euler_to_dcm(float roll_deg, float pitch_deg, float yaw_deg, struct sensor_rot_matrix *dcm)
{
    float roll  = DEGREES_TO_RADIANS(roll_deg);
    float pitch = DEGREES_TO_RADIANS(pitch_deg);
    float yaw   = DEGREES_TO_RADIANS(yaw_deg);

    float sr = sinf(roll);
    float cr = cosf(roll);
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    dcm->m[0][0] = cp * cy;
    dcm->m[0][1] = cy * sp * sr - sy * cr;
    dcm->m[0][2] = cy * sp * cr + sy * sr;

	dcm->m[1][0] = cp * sy;
    dcm->m[1][1] = sy * sp * sr + cy * cr;
    dcm->m[1][2] = sy * sp * cr - cy * sr;

    dcm->m[2][0] = -sp;
    dcm->m[2][1] = cp * sr;
    dcm->m[2][2] = cp * cr;
}

void sensor_rotate_3f(enum sensor_rotation rot, float *x, float *y, float *z)
{
	if (sensor_rotate_direct(rot, x, y, z)==0) {
		if (rot < ROTATION_MAX) {
			struct sensor_rot_matrix dcm;
			euler_to_dcm((float)rot_lookup[rot].roll, (float)rot_lookup[rot].pitch, (float)rot_lookup[rot].yaw, &dcm);
			*x = dcm.m[0][0]*(*x) + dcm.m[0][1]*(*y) + dcm.m[0][2]*(*z);
			*y = dcm.m[1][0]*(*x) + dcm.m[1][1]*(*y) + dcm.m[1][2]*(*z);
			*z = dcm.m[2][0]*(*x) + dcm.m[2][1]*(*y) + dcm.m[2][2]*(*z);
		}
	}
}
