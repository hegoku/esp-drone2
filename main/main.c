#include <stdio.h>
#include "bus/bus_tree.h"

void app_main(void)
{
	init_bus();

	for(;;) {
		esp_rom_delay_us(50000);
	}
}
