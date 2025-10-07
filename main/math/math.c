#include <math.h>
#include "math/math.h"

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float calculate_variance(float samples[], int count)
{
	float avg = 0;
	for (int i=0;i<count;i++) {
		avg += samples[i];
	}
	avg = avg/count;
	float sum = 0;
	for (int i=0;i<count;i++) {
		sum += (samples[i] - avg) * (samples[i] - avg);
	}
	return sqrtf(sum/(count-1));
}