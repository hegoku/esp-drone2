#include "platform/gpio_config/gpio_config.h"
#include "mixer/mixer.h"

struct mixer_moter __motors[] = {
	{
		.wire = MOTOR1_GPIO
	},
	{
		.wire = MOTOR2_GPIO
	},
	{
		.wire = MOTOR3_GPIO
	},
	{
		.wire = MOTOR4_GPIO
	}
};