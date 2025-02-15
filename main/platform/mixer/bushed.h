#ifndef PLATFORM_MIXER_BUSHED_H
#define PLATFORM_MIXER_BUSHED_H

#include <driver/ledc.h>

struct bushed_protocol_motor_priv {
	ledc_channel_config_t channel;
};

#define BUSHED_GET_MOTOR_PRIV(x) ((struct bushed_protocol_motor_priv*)x)
#endif