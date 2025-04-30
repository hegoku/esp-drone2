#include <math.h>
#include "misc/geo.h"
#include "math/math.h"
#include "math/quaternion.h"
#include "attitude/estimator_q.h"

const float bias_max = 0.05f;

void estimator_q(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, struct estimator_q_s *res)
{
	float corr_x, corr_y, corr_z;
	struct quaternion q = {res->q0, res->q1, res->q2, res->q3};
	struct quaternion mag = {0, mx, my, mz};
	struct quaternion mag_earth = {0};
	quat_rotate_vector(&q, &mag, &mag_earth);

	float mag_err = atan2f(mag_earth.q2, mag_earth.q1);

	float gainMult = 1.0f;
	const float fifty_dps = 0.873f;
	float spinRate = sqrtf(gx*gx+gy*gy+gz*gz);

	if (spinRate > fifty_dps) {
		gainMult = min(spinRate / fifty_dps, 10.0f);
	}

	mag.q1=0.0f;
	mag.q2=0.0f;
	mag.q3=-mag_err;
	quat_rotate_vector_inverse(&q, &mag, &mag_earth);
	corr_x = mag_earth.q1 * gainMult * res->w_mag;
	corr_y = mag_earth.q2 * gainMult * res->w_mag;
	corr_z = mag_earth.q3 * gainMult * res->w_mag;

	const float accel_norm_sq = ax*ax+ay*ay+az*az;
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

	float recipNorm = invSqrt(accel_norm_sq);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	float halfvx = res->q1 * res->q3 - res->q0 * res->q2;
	float halfvy = res->q0 * res->q1 + res->q2 * res->q3;
	float halfvz = res->q0 * res->q0 - 0.5f + res->q3 * res->q3;

	// Error is sum of cross product between estimated and measured direction of gravity
	float halfex = (ay * halfvz - az * halfvy);
	float halfey = (az * halfvx - ax * halfvz);
	float halfez = (ax * halfvy - ay * halfvx);

	if ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
		(accel_norm_sq < upper_accel_limit * upper_accel_limit)) {
		corr_x += halfex * res->w_accel;
		corr_y += halfey * res->w_accel;
		corr_z += halfez * res->w_accel;
	}

	if (spinRate<0.175f) {
		res->gyro_biax_x += corr_x * res->w_gyro_bias * res->dt;
		res->gyro_biax_y += corr_y * res->w_gyro_bias * res->dt;
		res->gyro_biax_z += corr_z * res->w_gyro_bias * res->dt;

		res->gyro_biax_x = constrain(res->gyro_biax_x, -bias_max, bias_max);
		res->gyro_biax_y = constrain(res->gyro_biax_y, -bias_max, bias_max);
		res->gyro_biax_z = constrain(res->gyro_biax_z, -bias_max, bias_max);
	}

	gx += corr_x + res->gyro_biax_x;
	gy += corr_y + res->gyro_biax_y;
	gz += corr_z + res->gyro_biax_z;

	gx *= (0.5f * res->dt);		// pre-multiply common factors
	gy *= (0.5f * res->dt);
	gz *= (0.5f * res->dt);
	float qa = res->q0;
	float qb = res->q1;
	float qc = res->q2;
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