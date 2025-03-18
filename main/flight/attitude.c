#include <math.h>
#include "flight/flight.h"
#include "attitude/MahonyAHRS.h"

static struct mahony_ahrs mahony;

void init_attitude()
{
	if (flight.imu.status!=IMU_STATUS_ON)
		return;
	mahony.q0 = 1.0f;
	mahony.q1 = 0.0f;
	mahony.q2 = 0.0f;
	mahony.q3 = 0.0f;
	mahony.twoKp = 409.5f ;
	mahony.twoKi = 0.002f ;
	mahony.sampleFreq = (float)flight.imu.freq;
	mahony.integralFBx = 0.0f;
	mahony.integralFBy = 0.0f;
	mahony.integralFBz = 0.0f;
}

void calculate_attitude()
{
	if (flight.imu.status!=IMU_STATUS_ON)
		return;
	// MahonyAHRSupdate(flight.imu.gyro.value.x, flight.imu.gyro.value.y, flight.imu.gyro.value.z, flight.imu.accel.value.x, flight.imu.accel.value.y, flight.imu.accel.value.z, flight.compass.value.x, flight.compass.value.y, flight.compass.value.z, &mahony);
	MahonyAHRSupdateIMU(flight.imu.gyro.value.x, flight.imu.gyro.value.y, flight.imu.gyro.value.z, flight.imu.accel.value.x, flight.imu.accel.value.y, flight.imu.accel.value.z, &mahony);

	flight.attitude.q0 = mahony.q0;
	flight.attitude.q1 = mahony.q1;
	flight.attitude.q2 = mahony.q2;
	flight.attitude.q3 = mahony.q3;

	flight.attitude.roll = atan2f(2.0f*(flight.attitude.q0*flight.attitude.q1+ flight.attitude.q2*flight.attitude.q3), 1.0f- 2.0f * (flight.attitude.q1*flight.attitude.q1 + flight.attitude.q2*flight.attitude.q2))* 57.2958f;
	flight.attitude.pitch = -asinf(2.0f*(flight.attitude.q1*flight.attitude.q3 - flight.attitude.q0*flight.attitude.q2))* 57.2958f;
	if (IS_COMPASS_DTRY(flight.compass)) {
		// flight.attitude.yaw = atan2f(2.0f*(flight.attitude.q1*flight.attitude.q2 + flight.attitude.q0*flight.attitude.q3), 1.0f - 2.0f *(flight.attitude.q2*flight.attitude.q2 + flight.attitude.q3*flight.attitude.q3))* 57.2958f;
	}
}