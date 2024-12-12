#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "drivers/bus/bus_tree.h"

void app_main(void)
{
	init_bus_tree();

	for(;;) {
		esp_rom_delay_us(100000);
	}
}
