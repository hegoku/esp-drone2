#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/time.h>
#include <esp_log.h>
#include "platform/bus/bus_tree.h"
#include "clocksource/clocksource.h"
#include "clocksource/default_source.h"
#include "flight/flight.h"
#include "task/task.h"
#include "misc/config.h"
#include "platform/drivers/wifi.h"
#include "anotc/anotc_official_frame.h"

QueueHandle_t sys_timer_queue;
struct timeval sys_timer_time;
struct timeval finish_loop;

int anotc_log(const char * format, va_list arg)
{
	char subf[256];
	int actual_size = 0;
	actual_size = vsnprintf(subf, 256, format, arg);
	if (actual_size>=0) {
		anotc_send_log_string(BLACK, subf, actual_size);
	}
	return actual_size;
}

void app_main(void)
{
	init_config();
	init_wifi();
	sys_timer_set(&default_timer);
	
	init_bus_tree();
	print_bus_tree();
	
	esp_log_set_vprintf(anotc_log);

	init_flight();

	sys_timer_queue = xQueueCreate(5, sizeof( struct timeval ));
	vTaskPrioritySet(NULL, 10); 
	sys_timer_start();
	for (;;) {
		if (xQueueReceive(sys_timer_queue, &(sys_timer_time), portMAX_DELAY)){
			flight_read_data();
			flight_update();
			gettimeofday(&finish_loop, NULL);
			flight.system_info.cpu_load = (((float)finish_loop.tv_sec) + ((float)finish_loop.tv_usec) / 1000000.0 - ((float)sys_timer_time.tv_sec) + ((float)sys_timer_time.tv_usec) / 1000000.0) * (float)(sys_timer_get()->freq);
			log_task();
		}
	} 
}
