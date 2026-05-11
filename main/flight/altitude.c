#include "flight/altitude.h"
#include "flight/flight.h"
#include "math/quaternion.h"
#include "misc/geo.h"
#include "math/matrix.h"

#define ALTITUDE_DEFAULT_BARO_VARIANCE 0.10f
#define ALTITUDE_DEFAULT_ACCEL_NOISE 0.50f
#define ALTITUDE_DEFAULT_BIAS_NOISE 0.03f

struct altitude_kalman_filter_param {
	float dt;
	float gravity;
	float accel_noise;
	float bias_noise;
	float prior_x[3][1];
	float x_hat[3][1];
	float prior_P[3][3];
	float P[3][3];
	float R[1][1];
	float K[3][1];

	float A[3][3];
	float H[1][3];
	float Q[3][3];
	float G[3][1];
};

struct altitude_kalman_filter_param f = {
	.dt=0.001,
	.gravity=CONSTANTS_ONE_G,
	.accel_noise=ALTITUDE_DEFAULT_ACCEL_NOISE,
	.bias_noise=ALTITUDE_DEFAULT_BIAS_NOISE,
	.A={{0,0,0}, {0,0,0}, {0,0,0} },
	.prior_x={{0}, {0},{0}},
	.x_hat={{0}, {0},{0}},
	.prior_P={{0,0,0}, {0,0,0}, {0,0,0}},
	.R={{ALTITUDE_DEFAULT_BARO_VARIANCE}},
	.P={{100.0f, 0.0f, 0}, {0.0f, 100.0f,0}, {0,0,100.0f}},
	.H={{1.0f,0,0}},
	.Q={{0,0,0}, {0,0,0}, {0,0,0}},
	.G={{0},{0},{0}}
};

void altitude_kalman_filter(float accel, float height, struct altitude_kalman_filter_param *filter)
{
	float dt2 = filter->dt * filter->dt;
	float dt3 = dt2 * filter->dt;
	float dt4 = dt2 * dt2;
	float accel_variance = filter->accel_noise * filter->accel_noise;
	float bias_variance = filter->bias_noise * filter->bias_noise;
	float A_T[3][3] = {0};
	float H_T[3][1] = {0};
	filter->A[0][0] = 1.0f;
	filter->A[0][1] = filter->dt;
	filter->A[0][2] = -0.5f*dt2;
	filter->A[1][1] = 1.0f;
	filter->A[1][2] = -filter->dt;
	filter->A[2][2] = 1.0f;
	matrix_t(*(filter->A), 3,3, *A_T);
	matrix_t(*(filter->H), 1, 3, *H_T);

	filter->G[0][0] = 0.5f*dt2;
	filter->G[1][0] = filter->dt;

	filter->Q[0][0] = 0.25f * dt4 * accel_variance;
	filter->Q[0][1] = 0.5f * dt3 * accel_variance;
	filter->Q[0][2] = 0.0f;
	filter->Q[1][0] = filter->Q[0][1];
	filter->Q[1][1] = dt2 * accel_variance;
	filter->Q[1][2] = 0.0f;
	filter->Q[2][0] = 0.0f;
	filter->Q[2][1] = 0.0f;
	filter->Q[2][2] = filter->dt * bias_variance;

	accel-=filter->gravity;

	filter->G[0][0] = 0.5f*dt2*accel;
	filter->G[1][0] = filter->dt*accel;

	float tmp2[3][3] = {0};
	float tmp3[3][3] = {0};

	matrix_mult(*(filter->A), 3,3, *(filter->P), 3,3, *tmp2);
	matrix_mult(*tmp2, 3,3, *A_T, 3,3, *tmp3);
	matrix_add(*tmp3, *(filter->Q), *(filter->prior_P), 3,3);

	matrix_clear(*(filter->prior_x), 3,1);
	matrix_mult(*(filter->A), 3,3, *(filter->x_hat), 3,1, *(filter->prior_x));
	matrix_add(*(filter->G), *(filter->prior_x), *(filter->prior_x), 3, 1);

	if (!IS_BARO_DTRY(flight.baro)) {
		filter->x_hat[0][0] = filter->prior_x[0][0];
		filter->x_hat[1][0] = filter->prior_x[1][0];
		filter->x_hat[2][0] = filter->prior_x[2][0];

		filter->P[0][0] = filter->prior_P[0][0];
		filter->P[0][1] = filter->prior_P[0][1];
		filter->P[0][2] = filter->prior_P[0][2];
		filter->P[1][0] = filter->prior_P[1][0];
		filter->P[1][1] = filter->prior_P[1][1];
		filter->P[1][2] = filter->prior_P[1][2];
		filter->P[2][0] = filter->prior_P[2][0];
		filter->P[2][1] = filter->prior_P[2][1];
		filter->P[2][2] = filter->prior_P[2][2];
		return;
	}

	//cal K gain
	float HP[1][3] = {0};
	float HPHT[1][1] = {0};
	matrix_mult(*(filter->H), 1,3, *(filter->prior_P), 3,3, *HP);
	matrix_mult(*HP, 1,3, *H_T, 3, 1, *HPHT);
	matrix_add(*HPHT, *(filter->R), *HPHT, 1, 1);
	//inverse HPHT_R
	float HPHT_R[1][1] = {0};
	HPHT_R[0][0] = 1.0f / HPHT[0][0];
	// matrix_inverse(HPHT, 1, HPHT_R);
	float PH[3][1] = {0};
	matrix_clear(*(filter->K), 3,1);
	matrix_mult(*(filter->prior_P), 3,3, *H_T, 3,1, *PH);
	matrix_mult(*PH, 3,1, *HPHT_R, 1,1, *(filter->K));

	//estimate state
	float HX[1][1] = {0};
	matrix_mult(*(filter->H), 1,3, *(filter->prior_x), 3,1, *HX);
	HX[0][0] = height - HX[0][0];
	matrix_clear(*(filter->x_hat), 3,1);
	matrix_mult(*(filter->K), 3,1, *HX, 1,1, *(filter->x_hat));
	matrix_add(*(filter->x_hat), *(filter->prior_x), *(filter->x_hat), 3, 1);

	//update estimate uncertainty
	matrix_clear(*tmp2, 3,3); //KH
	float I[3][3] = {{1.0f, 0.0f, 0}, {0.0f, 1.0f,0}, {0,0,1.0f}};
	matrix_mult(*(filter->K), 3,1, *(filter->H), 1,3, *tmp2);
	matrix_sub(*I, *tmp2, *tmp3, 3,3);
	float I_KH_T[3][3] = {0};
	matrix_t(*tmp3, 3,3, *I_KH_T);
	matrix_clear(*tmp2, 3,3);
	matrix_mult(*tmp3, 3,3, *(filter->prior_P), 3,3, *tmp2);
	matrix_clear(*(filter->P), 3,3);
	matrix_mult(*tmp2, 3,3, *I_KH_T, 3,3, *(filter->P));
	float KR[3][1] = {0};
	float K_T[1][3] = {0};
	matrix_t(*(filter->K), 3,1, *K_T);
	matrix_mult(*(filter->K), 3,1, *(filter->R), 1,1, *KR);
	matrix_clear(*tmp2, 3,3);
	matrix_mult(*KR, 3,1, *K_T, 1,3, *tmp2);
	matrix_add(*(filter->P), *tmp2, *(filter->P), 3,3);
}

void calculate_altitude()
{
	if (!IS_BARO_ON(flight.baro))
		return;
	struct quaternion q = {flight.attitude.q0, flight.attitude.q1, flight.attitude.q2, flight.attitude.q3};
	float dcm[3][3];
	quat_2_dcm(&q, dcm);
	flight.acceleration.x = flight.imu.accel.value.x * dcm[0][0] + flight.imu.accel.value.y * dcm[0][1] + flight.imu.accel.value.z * dcm[0][2];
	flight.acceleration.y = flight.imu.accel.value.x * dcm[1][0] + flight.imu.accel.value.y * dcm[1][1] + flight.imu.accel.value.z * dcm[1][2];
	flight.acceleration.z = flight.imu.accel.value.x * dcm[2][0] + flight.imu.accel.value.y * dcm[2][1] + flight.imu.accel.value.z * dcm[2][2];

	altitude_kalman_filter(flight.acceleration.z, flight.baro.altitude, &f);
	flight.velocity.z = f.x_hat[1][0];
	flight.altitude = f.x_hat[0][0];
	if (IS_BARO_DTRY(flight.baro)) {
		// flight.altitude = flight.baro.altitude;
	}
}

float altitude_get_gravity()
{
	return f.gravity;
}

float altitude_get_accel_bias()
{
	return f.x_hat[2][0];
}

float altitude_get_k_altitude()
{
	return f.K[0][0];
}

float altitude_get_k_velocity()
{
	return f.K[1][0];
}

float altitude_get_k_accel_bias()
{
	return f.K[2][0];
}
