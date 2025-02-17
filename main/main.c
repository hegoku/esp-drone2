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
#include "platform/drivers/wifi.h"
#include "platform/drivers/anotc_wifi.h"
#include "anotc/anotc_official_frame.h"

QueueHandle_t sys_timer_queue;
QueueHandle_t log_task_queue;
struct timeval sys_timer_time;
struct timeval finish_loop;

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
	for (;;)
	{
		if (xQueueReceive(sys_timer_queue, &(a), portMAX_DELAY)){
			gpio_set_level(GPIO_NUM_12, b);
			b = ~b;
			gettimeofday(&sys_timer_time, NULL);
			flight_read_data();
			flight_update();
			flight_control();
			gettimeofday(&finish_loop, NULL);
			flight.system_info.cpu_load = (unsigned char)((((float)finish_loop.tv_sec) + ((float)finish_loop.tv_usec) / 1000000.0 - ((float)sys_timer_time.tv_sec) - ((float)sys_timer_time.tv_usec) / 1000000.0) * ((float)(sys_timer_get()->freq)) * 100.0);
			xQueueSend(log_task_queue, &a, 0);
			gpio_set_level(GPIO_NUM_12, b);
			b = ~b;
		}
	}
}

void log_t(void *p)
{
	int a;
	for (;;)
	{
		if (xQueueReceive(log_task_queue, &(a), portMAX_DELAY)){
			flight_log_task();
			wifi_task();
		}
	}
}

void app_main(void)
{
	init_config();
	sys_timer_set(&default_timer);
	// if (init_wifi()==0) {
	// 	anotc_wifi_init();
	// }
	
	init_bus_tree();
	print_bus_tree();
	
	esp_log_set_vprintf(anotc_log);

	init_flight();

	sys_timer_queue = xQueueCreate(5, sizeof( int ));
	xTaskCreatePinnedToCore(main_loop, "main_loop", 1024*4, NULL, 10, NULL, 1);

	log_task_queue = xQueueCreate(5, sizeof( int ));
	xTaskCreatePinnedToCore(log_t, "log_task", 1024*4, NULL, 5, NULL, 0);

	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << GPIO_NUM_12),
		.pull_down_en = 0,
		.pull_up_en = 0
	};
	gpio_config(&io_conf);
	
	sys_timer_start();
}
