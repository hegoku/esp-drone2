#ifndef ATTITUDE_ESTIMATOR_Q_H
#define ATTITUDE_ESTIMATOR_Q_H

struct estimator_q_s {
	float q0,q1,q2,q3;
	float w_accel;
	float w_mag;
	float w_gyro_bias;
	float dt;
	float gyro_biax_x;
	float gyro_biax_y;
	float gyro_biax_z;
};

void estimator_q(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, struct estimator_q_s *res);
#endif