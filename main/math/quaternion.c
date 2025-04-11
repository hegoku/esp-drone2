#include <math.h>
#include <string.h>
#include "math/math.h"
#include "math/quaternion.h"

void vector_cross(float mat1[3], float mat2[3], float res[3])
{
	res[0] = mat1[1]*mat2[2] - mat1[2]*mat2[1];
	res[1] = mat1[2]*mat2[0] - mat1[0]*mat2[2];
	res[2] = mat1[0]*mat2[1] - mat1[1]*mat2[0];
}

float vector_angle(float mat1[3], float mat2[3])
{
	float product = mat1[0]*mat2[0] + mat1[1]*mat2[1] + mat1[2]*mat2[2];
	float a = invNormal(mat1) * invNormal(mat2);
	float cos_angle = product * a;
	return acosf(cos_angle);
}

void quat_normalize(struct quaternion *q, struct quaternion *n_q)
{
	float norm = invSqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
	n_q->q0 = q->q0 * norm;
	n_q->q1 = q->q1 * norm;
	n_q->q2 = q->q2 * norm;
	n_q->q3 = q->q3 * norm;
}

void quat_dcm_z(struct quaternion *q, float mat[3])
{
	mat[0] = 2.0f * (q->q0 * q->q2 + q->q1 * q->q3);
	mat[1] = 2.0f * (q->q2 * q->q3 - q->q0 * q->q1);
	mat[2] = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
}

void quat_dcm_z_T(struct quaternion *q, float mat[3])
{
	mat[0] = 2.0f * (q->q1 * q->q3 - q->q0 * q->q2);
	mat[1] = 2.0f * (q->q2 * q->q3 + q->q0 * q->q1);
	mat[2] = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
}

void quat_from_vector(float mat1[3], float mat2[3], struct quaternion *res)
{
	float half_angle = vector_angle(mat1, mat2) / 2.0f;
	float cross[3];
	float sin_half_angle = sinf(half_angle);
	vector_cross(mat1, mat2, cross);
	float inv_normal = invNormal(cross);
	cross[0] *= inv_normal;
	cross[1] *= inv_normal;
	cross[2] *= inv_normal;
	res->q0 = cosf(half_angle);
	res->q1 = sin_half_angle * cross[0];
	res->q2 = sin_half_angle * cross[1];
	res->q3 = sin_half_angle * cross[2];
}

void quat_product(struct quaternion *q1, struct quaternion *q2)
{
	struct quaternion m;
	m.q0 = q1->q0*q2->q0 - q1->q1*q2->q1 - q1->q2*q2->q2 - q1->q3*q2->q3;
	m.q1 = q1->q1*q2->q0 + q1->q0*q2->q1 - q1->q3*q2->q2 + q1->q2*q2->q3;
	m.q2 = q1->q2*q2->q0 + q1->q3*q2->q1 + q1->q0*q2->q2 - q1->q1*q2->q3;
	m.q3 = q1->q3*q2->q0 - q1->q2*q2->q1 + q1->q1*q2->q2 + q1->q0*q2->q3;
	memcpy(q1, &m, sizeof(struct quaternion));
}

void quat_inverse(struct quaternion *q, struct quaternion *res)
{
	float norm = invSqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
	res->q0 = q->q0 * norm;
	res->q1 = -q->q1 * norm;
	res->q2 = -q->q2 * norm;
	res->q3 = -q->q3 * norm;
}

void euler_2_quat(float roll, float pitch, float yaw, struct quaternion *q)
{
	float y = yaw/180.0f*M_PI*0.5f;
	float p = pitch/180.0f*M_PI*0.5f;
	float r = roll/180.0f*M_PI*0.5f;
	float cy = cosf(y);
	float sy = sinf(y);
	float cp = cosf(p);
	float sp = sinf(p);
	float cr = cosf(r);
	float sr = sinf(r);

	q->q0 = cy*cp*cr + sy*sp*sr;
	q->q1 = cy*cp*sr - sy*sp*cr;
	q->q2 = sy*cp*sr + cy*sp*cr;
	q->q3 = sy*cp*cr - cy*sp*sr;
}

void quat_2_dcm(struct quaternion *q, float mat[3][3])
{
	mat[0][0] = 1.0f - 2.0f * (q->q2*q->q2 + q->q3*q->q3);
	mat[1][0] = 2.0f * (q->q1*q->q2 + q->q0*q->q3);
	mat[2][0] = 2.0f * (q->q1*q->q3 - q->q0*q->q2);

	mat[0][1] = 2.0f * (q->q1*q->q2 - q->q0*q->q3);
	mat[1][1] = 1.0f - 2.0f * (q->q1*q->q1 + q->q3*q->q3);
	mat[2][1] = 2.0f * (q->q2*q->q3 + q->q0*q->q1);

	mat[0][2] = 2.0f * (q->q0 * q->q2 + q->q1 * q->q3);
	mat[1][2] = 2.0f * (q->q2 * q->q3 - q->q0 * q->q1);
	mat[2][2] = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
}