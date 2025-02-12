#ifndef PLATFORM_ESC_DSHOT_H
#define PLATFORM_ESC_DSHOT_H

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include "esc/dshot.h"

struct dshot_protocol_motor {
	gpio_num_t gpio_num;
	rmt_channel_handle_t *rmt_channel;
	rmt_encoder_handle_t rmt_encoder;
};

#define DSHOT_GET_MOTOR_PRIV(x) ((struct dshot_protocol_motor*)x)

void dshot_init(struct dshot_protocol_motor *dshot);
void dshot_write(struct dshot_protocol_motor *dshot, unsigned short value, unsigned char telemetry);

#endif