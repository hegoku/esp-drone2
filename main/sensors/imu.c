#include <stdio.h>
#include "imu.h"
#include "flight/flight.h"

#include "drivers/imu/icm20948.h"

extern struct imu_sensor icm20948;

void init_imu()
{
	if (icm20948.dev->init(icm20948.dev)==0) {
		printf("%s init success\n", icm20948.name);
		flight.imu = &icm20948;
	}
}