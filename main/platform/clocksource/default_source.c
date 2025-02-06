#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include "clocksource/clocksource.h"
#include "clocksource/default_source.h"

extern QueueHandle_t sys_timer_queue;

esp_timer_handle_t periodic_timer;

static void default_sys_timer(void *arg)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	xQueueSend(sys_timer_queue, &tv, 0);
}

void default_timer_start()
{
	esp_timer_create_args_t periodic_timer_args = {
            .callback = &default_sys_timer,
            .name = "sys_timer"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));
}

struct clocksource default_timer = {
	.name = "default",
	.freq = 1000,
	.start = default_timer_start
};