#include <driver/gpio.h>
#include "mixer/mixer.h"

struct mixer_moter __motors[] = {
	{
		.wire = GPIO_NUM_25
	},
	{
		.wire = GPIO_NUM_26
	},
	{
		.wire = GPIO_NUM_27
	},
	{
		.wire = GPIO_NUM_14
	}
};