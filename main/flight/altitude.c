#include "flight/altitude.h"
#include "flight/flight.h"
#include "math/quaternion.h"
#include "misc/geo.h"
#include "math/matrix.h"

#define ALTITUDE_DEFAULT_BARO_VARIANCE 0.10f
#define ALTITUDE_DEFAULT_ACCEL_NOISE 0.50f
#define ALTITUDE_DEFAULT_BIAS_NOISE 0.03f
#define ALTITUDE_DEFAULT_BARO_BIAS_NOISE 0.03f
#define ALTITUDE_STATE_SIZE 4

struct altitude_kalman_filter_param {
	float dt;
	float gravity;
	float accel_noise;
	float bias_noise;
	float baro_bias_noise;
	float prior_x[ALTITUDE_STATE_SIZE][1];
	float x_hat[ALTITUDE_STATE_SIZE][1];
	float prior_P[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE];
	float P[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE];
	float R[1][1];
	float K[ALTITUDE_STATE_SIZE][1];

	float A[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE];
	float H[1][ALTITUDE_STATE_SIZE];
	float Q[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE];
	float G[ALTITUDE_STATE_SIZE][1];
};

struct altitude_kalman_filter_param f = {
	.dt=0.001,
	.gravity=CONSTANTS_ONE_G,
	.accel_noise=ALTITUDE_DEFAULT_ACCEL_NOISE,
	.bias_noise=ALTITUDE_DEFAULT_BIAS_NOISE,
	.baro_bias_noise=ALTITUDE_DEFAULT_BARO_BIAS_NOISE,
	.A={{0}},
	.prior_x={{0}, {0}, {0}, {0}},
	.x_hat={{0}, {0}, {0}, {0}},
	.prior_P={{0}},
	.R={{ALTITUDE_DEFAULT_BARO_VARIANCE}},
	.P={{100.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, 100.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 100.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 0.01f}},
	.H={{1.0f, 0.0f, 0.0f, 1.0f}},
	.Q={{0}},
	.G={{0}, {0}, {0}, {0}}
};

void altitude_kalman_filter(float accel, float height, struct altitude_kalman_filter_param *filter)
{
	float dt2 = filter->dt * filter->dt;
	float dt3 = dt2 * filter->dt;
	float dt4 = dt2 * dt2;
	float accel_variance = filter->accel_noise * filter->accel_noise;
	float bias_variance = filter->bias_noise * filter->bias_noise;
	float baro_bias_variance = filter->baro_bias_noise * filter->baro_bias_noise;
	float A_T[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE] = {0};
	float H_T[ALTITUDE_STATE_SIZE][1] = {0};
	matrix_clear(*(filter->A), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	filter->A[0][0] = 1.0f;
	filter->A[0][1] = filter->dt;
	filter->A[0][2] = -0.5f*dt2;
	filter->A[1][1] = 1.0f;
	filter->A[1][2] = -filter->dt;
	filter->A[2][2] = 1.0f;
	filter->A[3][3] = 1.0f;
	matrix_t(*(filter->A), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *A_T);
	matrix_t(*(filter->H), 1, ALTITUDE_STATE_SIZE, *H_T);

	filter->G[0][0] = 0.5f*dt2;
	filter->G[1][0] = filter->dt;
	filter->G[2][0] = 0.0f;
	filter->G[3][0] = 0.0f;

	matrix_clear(*(filter->Q), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	filter->Q[0][0] = 0.25f * dt4 * accel_variance;
	filter->Q[0][1] = 0.5f * dt3 * accel_variance;
	filter->Q[1][0] = filter->Q[0][1];
	filter->Q[1][1] = dt2 * accel_variance;
	filter->Q[2][2] = filter->dt * bias_variance;
	filter->Q[3][3] = filter->dt * baro_bias_variance;

	accel-=filter->gravity;

	filter->G[0][0] = 0.5f*dt2*accel;
	filter->G[1][0] = filter->dt*accel;

	float tmp2[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE] = {0};
	float tmp3[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE] = {0};

	matrix_mult(*(filter->A), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *(filter->P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *tmp2);
	matrix_mult(*tmp2, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *A_T, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *tmp3);
	matrix_add(*tmp3, *(filter->Q), *(filter->prior_P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);

	matrix_clear(*(filter->prior_x), ALTITUDE_STATE_SIZE, 1);
	matrix_mult(*(filter->A), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *(filter->x_hat), ALTITUDE_STATE_SIZE, 1, *(filter->prior_x));
	matrix_add(*(filter->G), *(filter->prior_x), *(filter->prior_x), ALTITUDE_STATE_SIZE, 1);

	if (!IS_BARO_DTRY(flight.baro)) {
		for (int i = 0; i < ALTITUDE_STATE_SIZE; i++) {
			filter->x_hat[i][0] = filter->prior_x[i][0];
			for (int j = 0; j < ALTITUDE_STATE_SIZE; j++) {
				filter->P[i][j] = filter->prior_P[i][j];
			}
		}
		return;
	}

	//cal K gain
	float HP[1][ALTITUDE_STATE_SIZE] = {0};
	float HPHT[1][1] = {0};
	matrix_mult(*(filter->H), 1, ALTITUDE_STATE_SIZE, *(filter->prior_P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *HP);
	matrix_mult(*HP, 1, ALTITUDE_STATE_SIZE, *H_T, ALTITUDE_STATE_SIZE, 1, *HPHT);
	matrix_add(*HPHT, *(filter->R), *HPHT, 1, 1);
	//inverse HPHT_R
	float HPHT_R[1][1] = {0};
	HPHT_R[0][0] = 1.0f / HPHT[0][0];
	// matrix_inverse(HPHT, 1, HPHT_R);
	float PH[ALTITUDE_STATE_SIZE][1] = {0};
	matrix_clear(*(filter->K), ALTITUDE_STATE_SIZE, 1);
	matrix_mult(*(filter->prior_P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *H_T, ALTITUDE_STATE_SIZE, 1, *PH);
	matrix_mult(*PH, ALTITUDE_STATE_SIZE, 1, *HPHT_R, 1, 1, *(filter->K));

	//estimate state
	float HX[1][1] = {0};
	matrix_mult(*(filter->H), 1, ALTITUDE_STATE_SIZE, *(filter->prior_x), ALTITUDE_STATE_SIZE, 1, *HX);
	HX[0][0] = height - HX[0][0];
	matrix_clear(*(filter->x_hat), ALTITUDE_STATE_SIZE, 1);
	matrix_mult(*(filter->K), ALTITUDE_STATE_SIZE, 1, *HX, 1, 1, *(filter->x_hat));
	matrix_add(*(filter->x_hat), *(filter->prior_x), *(filter->x_hat), ALTITUDE_STATE_SIZE, 1);

	//update estimate uncertainty
	matrix_clear(*tmp2, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE); //KH
	float I[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE] = {{1.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}};
	matrix_mult(*(filter->K), ALTITUDE_STATE_SIZE, 1, *(filter->H), 1, ALTITUDE_STATE_SIZE, *tmp2);
	matrix_sub(*I, *tmp2, *tmp3, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	float I_KH_T[ALTITUDE_STATE_SIZE][ALTITUDE_STATE_SIZE] = {0};
	matrix_t(*tmp3, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *I_KH_T);
	matrix_clear(*tmp2, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	matrix_mult(*tmp3, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *(filter->prior_P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *tmp2);
	matrix_clear(*(filter->P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	matrix_mult(*tmp2, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *I_KH_T, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE, *(filter->P));
	float KR[ALTITUDE_STATE_SIZE][1] = {0};
	float K_T[1][ALTITUDE_STATE_SIZE] = {0};
	matrix_t(*(filter->K), ALTITUDE_STATE_SIZE, 1, *K_T);
	matrix_mult(*(filter->K), ALTITUDE_STATE_SIZE, 1, *(filter->R), 1, 1, *KR);
	matrix_clear(*tmp2, ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
	matrix_mult(*KR, ALTITUDE_STATE_SIZE, 1, *K_T, 1, ALTITUDE_STATE_SIZE, *tmp2);
	matrix_add(*(filter->P), *tmp2, *(filter->P), ALTITUDE_STATE_SIZE, ALTITUDE_STATE_SIZE);
}

void calculate_altitude()
{
	if (!IS_BARO_ON(flight.baro))
		return;
	float dcm[3][3];
	quat_2_dcm(&flight.attitude.q, dcm);
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

float altitude_get_baro_bias()
{
	return f.x_hat[3][0];
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

float altitude_get_k_baro_bias()
{
	return f.K[3][0];
}
