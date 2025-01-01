#include "bus/bus.h"

#include "drivers/imu/mpu6050.h"

int (*i2c_dev_probs[BUS_MAX_PROBS_NR])(struct bus_dev *dev) = {
	mpu6050_prob
};