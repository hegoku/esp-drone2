#include "misc/util.h"

void delay_ms(int ms)
{
	esp_rom_delay_us(ms*1000);
}
