//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef ATTITUDE_MAHONYAHRS_H
#define ATTITUDE_MAHONYAHRS_H

//----------------------------------------------------------------------------------------------------
// Variable declaration

struct mahony_ahrs {
	float q0, q1, q2, q3;
	float twoKp; // 2 * proportional gain (Kp)
	float twoKi; // 2 * integral gain (Ki)
	float dt;
	float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki
};

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, struct mahony_ahrs *res);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, struct mahony_ahrs *res);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
