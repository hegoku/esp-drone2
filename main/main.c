#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <sys/time.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include "platform/bus/bus_tree.h"
#include "clocksource/clocksource.h"
#include "clocksource/default_source.h"
#include "flight/flight.h"
#include "task/task.h"
#include "misc/config.h"
#include "misc/util.h"
#include "platform/drivers/wifi.h"
#include "platform/drivers/anotc_wifi.h"
#include "anotc/anotc_official_frame.h"

QueueHandle_t sys_timer_queue;

int anotc_log(const char * format, va_list arg)
{
	char subf[256];
	int actual_size = 0;
	actual_size = vsnprintf(subf, 256, format, arg);
	if (actual_size>=0) {
		int offset = 0;
		if (subf[0]=='\033') { //去掉esp monitor颜色
			offset+=7;
			actual_size-=7+5;
		}
		anotc_send_log_string(BLACK, subf+offset, actual_size);
	}
	return actual_size;
}

void main_loop(void *p)
{
	int a;
	int b = 0;
	unsigned long long finish_time = 0;
	for (;;)
	{
		if (xQueueReceive(sys_timer_queue, &(a), portMAX_DELAY)){
			flight.system_info.time = get_timestamp_us();
			flight_read_data();
			flight_update();
			flight_control();
			flight_log_task();
			finish_time = get_timestamp_us();
			flight.system_info.cpu_load = (unsigned char)((float)(finish_time - flight.system_info.time) / 1000000.0f * (float)(sys_timer_get()->freq) * 100.0);
			b = ~b;
		}
	}
}

void app_main(void)
{
	init_config();
	sys_timer_set(&default_timer);

	init_bus_tree();
	// print_bus_tree();

	if (init_wifi()==0) {
		anotc_wifi_init();
	} else {
		esp_log_set_vprintf(anotc_log);
	}

	init_flight();

	sys_timer_queue = xQueueCreate(5, sizeof( int ));
	xTaskCreatePinnedToCore(main_loop, "main_loop", 1024*6, NULL, 20, NULL, 1);

	sys_timer_start();
}
