#include "flight/altitude.h"
#include "flight/flight.h"
#include "math/quaternion.h"
#include "misc/geo.h"

void calculate_altitude()
{
	struct quaternion q = {flight.attitude.q0, flight.attitude.q1, flight.attitude.q2, flight.attitude.q3};
	float mat[3];
	quat_dcm_z(&q, mat);
	flight.velocity.x = mat[0] * CONSTANTS_ONE_G;
	flight.velocity.y = mat[1] * CONSTANTS_ONE_G;
	flight.velocity.z = mat[2] * CONSTANTS_ONE_G;

	if (IS_BARO_DTRY(flight.baro)) {
		flight.altitude = flight.baro.altitude;
	}
}