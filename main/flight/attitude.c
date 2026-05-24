#include <math.h>
#include "math/math.h"
#include "flight/flight.h"
#include "attitude/MahonyAHRS.h"
#include <stdio.h>

static struct mahony_ahrs mahony;

void init_attitude()
{
	if (flight.imu.status!=IMU_STATUS_ON)
		return;
	mahony.q0 = 1.0f;
	mahony.q1 = 0.0f;
	mahony.q2 = 0.0f;
	mahony.q3 = 0.0f;
	mahony.twoKp = 10.0f;
	mahony.twoKi = 0.0f;
	mahony.dt = 1.0f/(float)flight.imu.freq;
	mahony.integralFBx = 0.0f;
	mahony.integralFBy = 0.0f;
	mahony.integralFBz = 0.0f;
}

float deg_min_to_decimal(int deg, int min) {
    if (deg < 0) {
        return deg - (float)min / 60.0f;
    } else {
        return deg + (float)min / 60.0f;
    }
}

void calculate_attitude()
{
	if (flight.imu.status!=IMU_STATUS_ON)
		return;
	// if (flight.status==FLIGHT_STATUS_ANGLE_MODE) {
		// mahony.twoKp = 0.7f;
	// } else {
		// mahony.twoKp = 10.0f;
	// }
	MahonyAHRSupdate(DEGREES_TO_RADIANS(flight.imu.gyro.value.x), DEGREES_TO_RADIANS(flight.imu.gyro.value.y), DEGREES_TO_RADIANS(flight.imu.gyro.value.z), flight.imu.accel.value.x, flight.imu.accel.value.y, flight.imu.accel.value.z, flight.magnetometer.value.x, flight.magnetometer.value.y, flight.magnetometer.value.z, &mahony);

	flight.attitude.q.q0 = mahony.q0;
	flight.attitude.q.q1 = mahony.q1;
	flight.attitude.q.q2 = mahony.q2;
	flight.attitude.q.q3 = mahony.q3;

	flight.attitude.roll = atan2f(2.0f*(flight.attitude.q.q0*flight.attitude.q.q1+ flight.attitude.q.q2*flight.attitude.q.q3), 1.0f- 2.0f * (flight.attitude.q.q1*flight.attitude.q.q1 + flight.attitude.q.q2*flight.attitude.q.q2));
	flight.attitude.roll = RADIANS_TO_DEGREES(flight.attitude.roll);
	flight.attitude.pitch = -asinf(2.0f*(flight.attitude.q.q1*flight.attitude.q.q3 - flight.attitude.q.q0*flight.attitude.q.q2));
	flight.attitude.pitch = RADIANS_TO_DEGREES(flight.attitude.pitch);
	flight.attitude.yaw = atan2f(2.0f*(flight.attitude.q.q1*flight.attitude.q.q2 + flight.attitude.q.q0*flight.attitude.q.q3), 1.0f - 2.0f *(flight.attitude.q.q2*flight.attitude.q.q2 + flight.attitude.q.q3*flight.attitude.q.q3));
	flight.attitude.yaw = RADIANS_TO_DEGREES(flight.attitude.yaw);
	if (flight.magnetometer.declination!=0.0f) {
		flight.attitude.yaw += flight.magnetometer.declination;
	}
}