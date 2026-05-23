#include <math.h>
#include "math/math.h"
#include "misc/config.h"
#include "flight/flight.h"
#include "mixer/mixer.h"
#include "mixer/dshot.h"
#include "mixer/bushed.h"

void init_mixer()
{
	unsigned char protocol = 0;
	unsigned long int motor_mapping = 0x3210;
	config_read_uchar("esc_protocol", &protocol);
	config_read_uint("motor_mapping", &motor_mapping);
	switch (protocol)
	{
	case 0:
		flight.mixer.esc_protocol = &mixer_bushed;
		break;
	case 1:
		flight.mixer.esc_protocol = &mixer_dshot600;
		break;
	default:
		flight.mixer.esc_protocol = &mixer_bushed;
		break;
	}
	
	for (int i=0;i<sizeof(__motors)/sizeof(__motors[0]);i++) {
		flight.mixer.motor[i] = &__motors[MOTOR_GET_INDEX(MOTOR_MAPPING_VALUE(motor_mapping, i))];
	}
	
	flight.mixer.esc_protocol->init(&flight.mixer);
}

void mixer_output(struct mixer *mixer)
{
	mixer->esc_protocol->write(mixer);
}

void update_flight_mixer(float roll, float pitch, float yaw)
{
	flight.mixer.motor[0]->value = constrain((unsigned short)round((flight.throttle - roll + pitch + yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[1]->value = constrain((unsigned short)round((flight.throttle - roll - pitch - yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[2]->value = constrain((unsigned short)round((flight.throttle + roll + pitch - yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
	flight.mixer.motor[3]->value = constrain((unsigned short)round((flight.throttle + roll - pitch + yaw)*1000.0f), FLIGHT_THROTTLE_MIN, FLIGHT_THROTTLE_MAX);
}