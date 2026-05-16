#include <freertos/FreeRTOS.h>
#include "misc/util.h"
#include "esp_timer.h"

void delay_ms(unsigned int ms)
{
	esp_rom_delay_us(ms*1000);
}

unsigned int get_timestamp_ms()
{
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

unsigned long long get_timestamp_us()
{
    return esp_timer_get_time();
}