#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/time.h>
#include "platform/bus/bus_tree.h"
#include "clocksource/clocksource.h"
#include "clocksource/default_source.h"
#include "flight/flight.h"
#include "task/task.h"

QueueHandle_t sys_timer_queue;
struct timeval sys_timer_time;

void app_main(void)
{
	sys_timer_set(default_timer);
	
	init_bus_tree();
	printf("init bus done\n");
	print_bus_tree();

	init_flight();

	sys_timer_queue = xQueueCreate(5, sizeof( struct timeval ));
	vTaskPrioritySet(NULL, 10); 
	sys_timer_start();
	for (;;) {
		if (xQueueReceive(sys_timer_queue, &(sys_timer_time), portMAX_DELAY)){
			flight_read_data();
			flight_update();
			log_task();
		}
	} 
}
