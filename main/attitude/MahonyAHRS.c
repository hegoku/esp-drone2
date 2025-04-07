//=====================================================================================================
// MahonyAHRS.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>
#include "math/math.h"
#include "attitude/MahonyAHRS.h"

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, struct mahony_ahrs *res) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, res);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = res->q0 * res->q0;
        q0q1 = res->q0 * res->q1;
        q0q2 = res->q0 * res->q2;
        q0q3 = res->q0 * res->q3;
        q1q1 = res->q1 * res->q1;
        q1q2 = res->q1 * res->q2;
        q1q3 = res->q1 * res->q3;
        q2q2 = res->q2 * res->q2;
        q2q3 = res->q2 * res->q3;
        q3q3 = res->q3 * res->q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(res->twoKi > 0.0f) {
			res->integralFBx += res->twoKi * halfex * (1.0f / res->sampleFreq);	// integral error scaled by Ki
			res->integralFBy += res->twoKi * halfey * (1.0f / res->sampleFreq);
			res->integralFBz += res->twoKi * halfez * (1.0f / res->sampleFreq);
			gx += res->integralFBx;	// apply integral feedback
			gy += res->integralFBy;
			gz += res->integralFBz;
		}
		else {
			res->integralFBx = 0.0f;	// prevent integral windup
			res->integralFBy = 0.0f;
			res->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += res->twoKp * halfex;
		gy += res->twoKp * halfey;
		gz += res->twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / res->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / res->sampleFreq));
	gz *= (0.5f * (1.0f / res->sampleFreq));
	qa = res->q0;
	qb = res->q1;
	qc = res->q2;
	res->q0 += (-qb * gx - qc * gy - res->q3 * gz);
	res->q1 += (qa * gx + qc * gz - res->q3 * gy);
	res->q2 += (qa * gy - qb * gz + res->q3 * gx);
	res->q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(res->q0 * res->q0 + res->q1 * res->q1 + res->q2 * res->q2 + res->q3 * res->q3);
	res->q0 *= recipNorm;
	res->q1 *= recipNorm;
	res->q2 *= recipNorm;
	res->q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, struct mahony_ahrs *res) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = res->q1 * res->q3 - res->q0 * res->q2;
		halfvy = res->q0 * res->q1 + res->q2 * res->q3;
		halfvz = res->q0 * res->q0 - 0.5f + res->q3 * res->q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(res->twoKi > 0.0f) {
			res->integralFBx += res->twoKi * halfex * (1.0f / res->sampleFreq);	// integral error scaled by Ki
			res->integralFBy += res->twoKi * halfey * (1.0f / res->sampleFreq);
			res->integralFBz += res->twoKi * halfez * (1.0f / res->sampleFreq);
			gx += res->integralFBx;	// apply integral feedback
			gy += res->integralFBy;
			gz += res->integralFBz;
		}
		else {
			res->integralFBx = 0.0f;	// prevent integral windup
			res->integralFBy = 0.0f;
			res->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += res->twoKp * halfex;
		gy += res->twoKp * halfey;
		gz += res->twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / res->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / res->sampleFreq));
	gz *= (0.5f * (1.0f / res->sampleFreq));
	qa = res->q0;
	qb = res->q1;
	qc = res->q2;
	res->q0 += (-qb * gx - qc * gy - res->q3 * gz);
	res->q1 += (qa * gx + qc * gz - res->q3 * gy);
	res->q2 += (qa * gy - qb * gz + res->q3 * gx);
	res->q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(res->q0 * res->q0 + res->q1 * res->q1 + res->q2 * res->q2 + res->q3 * res->q3);
	res->q0 *= recipNorm;
	res->q1 *= recipNorm;
	res->q2 *= recipNorm;
	res->q3 *= recipNorm;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
