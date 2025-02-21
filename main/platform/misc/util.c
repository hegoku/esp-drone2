#include <freertos/FreeRTOS.h>
#include <sys/time.h>
#include "misc/util.h"
#include <stdio.h>

void delay_ms(unsigned int ms)
{
	esp_rom_delay_us(ms*1000);
}

unsigned int get_timestamp_ms()
{
	struct timeval time;
	gettimeofday(&time, NULL);
	return time.tv_sec * 1000 + time.tv_usec / 1000;
}
