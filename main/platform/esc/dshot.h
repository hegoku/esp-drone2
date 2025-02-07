#ifndef PLATFORM_ESC_DSHOT_H
#define PLATFORM_ESC_DSHOT_H

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include "esc/dshot.h"

struct dshot_protocol {
	gpio_num_t gpio_num;
	rmt_channel_handle_t rmt_channel;
	rmt_encoder_handle_t rmt_encoder;
	// rmt_item32_t rmt_data[16];
	unsigned short t1h;
	unsigned short t1l;
	unsigned short t0h;
	unsigned short t0l;
};

#endif