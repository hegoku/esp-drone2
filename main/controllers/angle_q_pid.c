#include <math.h>
#include "math/math.h"
#include "controllers/angle_q_pid.h"

void angle_q_pid_compute(struct angle_q_pid_param *p)
{
	struct quaternion q_current_b_2_w_inv;
	quat_inverse(&p->q_current_b_2_w, &q_current_b_2_w_inv);

	quat_product(&q_current_b_2_w_inv, &p->q_desired_b_2_w, &p->q_error_b);
	if (p->q_error_b.q0<0) {
		p->q_error_b.q1 = -p->q_error_b.q1;
		p->q_error_b.q2 = -p->q_error_b.q2;
		p->q_error_b.q3 = -p->q_error_b.q3;
	}

	float s = invSqrt(p->q_error_b.q0*p->q_error_b.q0 + p->q_error_b.q1*p->q_error_b.q1 + p->q_error_b.q2*p->q_error_b.q2);

	float att_error_x, att_error_y, att_error_z = 0.0f;
	if (s > 1.0f/1e-6f) {
		att_error_x = RADIANS_TO_DEGREES(2.0f * p->q_error_b.q1);
		att_error_y = RADIANS_TO_DEGREES(2.0f * p->q_error_b.q2);
		att_error_z = RADIANS_TO_DEGREES(2.0f * p->q_error_b.q3);
	} else {
		float theta = 2.0f * atan2f(s, p->q_error_b.q0);
		att_error_x = RADIANS_TO_DEGREES(theta * p->q_error_b.q1 * s);
		att_error_y = RADIANS_TO_DEGREES(theta * p->q_error_b.q2 * s);
		att_error_z = RADIANS_TO_DEGREES(theta * p->q_error_b.q3 * s);
	}

	p->output_roll = p->K_roll * att_error_x;
	p->output_pitch = p->K_pitch * att_error_y;
	p->output_yaw = p->K_yaw * att_error_z;
}