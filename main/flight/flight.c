#include "flight/flight.h"

struct flight flight;

void init_flight()
{
	init_imu(&flight.imu);
}