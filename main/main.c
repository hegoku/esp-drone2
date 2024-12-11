#include <stdio.h>
#include "sensors/imu.h"
#include "bus/bus_tree.h"

#define PIN_NUM_MISO GPIO_NUM_16
#define PIN_NUM_MOSI GPIO_NUM_18
#define PIN_NUM_CLK GPIO_NUM_19

void app_main(void)
{
	init_bus();
	// init_spi(PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK);

	init_imu();
}
