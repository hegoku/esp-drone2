#include "mixer/dshot.h"
#include <math.h>

unsigned short dshot_convert_throttle(unsigned short throttle)
{
	unsigned short p_range = MIXER_DSHOT_THROTTLE_MAX - MIXER_DSHOT_THROTTLE_MIN;
	unsigned short t_range = MIXER_THROTTLE_MAX - MIXER_THROTTLE_MIN;
	float factor = ((float)p_range) / (float)t_range;
	return (unsigned short)round(((float)(throttle - MIXER_THROTTLE_MIN)) * factor) + MIXER_DSHOT_THROTTLE_MIN;
}