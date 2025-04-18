#ifndef PLATFORM_GPIO_CONFIG_H
#define PLATFORM_GPIO_CONFIG_H

#include "sdkconfig.h"

#ifdef CONFIG_ESP_DRONE_HW_VER_1
#include "gpio_config_hw1.h"
#elif defined(CONFIG_ESP_DRONE_HW_VER_2)
#include "gpio_config_hw2.h"
#endif

#endif