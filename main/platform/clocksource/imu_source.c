#include <sys/time.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "clocksource/clocksource.h"
#include "clocksource/imu_source.h"
#include "platform/gpio_config/gpio_config.h"

extern QueueHandle_t sys_timer_queue;

static void IRAM_ATTR imu_isr_handler(void* arg)
{
	int a = 0;
	xQueueSendFromISR(sys_timer_queue, &a, NULL);
}

void imu_timer_start()
{
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1ULL << IMU_INT_GPIO),
		.pull_down_en = 0,
		.pull_up_en = 1
	};
	gpio_config(&io_conf);
	gpio_set_intr_type(IMU_INT_GPIO, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(IMU_INT_GPIO, imu_isr_handler, (void*)IMU_INT_GPIO);
}

struct clocksource imu_timer = {
	.name = "IMU",
	.freq = 1000,
	.start = imu_timer_start
};
