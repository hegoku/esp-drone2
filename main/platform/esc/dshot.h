#ifndef PLATFORM_ESC_DSHOT_H
#define PLATFORM_ESC_DSHOT_H

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include "esc/dshot.h"

struct dshot_protocol {
	gpio_num_t gpio_num;
	rmt_channel_handle_t rmt_channel;
	rmt_encoder_handle_t rmt_encoder;
};

#endif