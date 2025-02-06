#include "flight/altitude.h"
#include "flight/flight.h"

void calculate_altitude()
{
	if (IS_BARO_DTRY(flight.baro)) {
		flight.altitude = flight.baro.altitude;
	}
}