#include "bus/bus.h"

#include "drivers/imu/mpu6500.h"
#include "drivers/imu/icm20948.h"
#include "drivers/barometer/bmp280.h"
#include "drivers/compass/lis3mdl.h"

int (*spi_dev_probs[BUS_MAX_PROBS_NR])(struct bus_dev *dev) = {
	mpu6500_spi_prob,
	icm20948_prob,
	bmp280_prob,
	lis3mdl_prob,
};