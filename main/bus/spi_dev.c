#include "bus/bus.h"

#include "drivers/imu/icm20948.h"

int (*spi_dev_probs[])(struct bus_dev *dev) = {
	icm20948_prob
};