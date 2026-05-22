#ifndef MATH_QUATERNION_H
#define MATH_QUATERNION_H

struct quaternion {
	float q0, q1, q2, q3;
};

void quat_normalize(struct quaternion *q, struct quaternion *n_q);
void quat_dcm_z(struct quaternion *q, float mat[3]);
void quat_dcm_z_T(struct quaternion *q, float mat[3]);
void quat_from_vector(float mat1[3], float mat2[3], struct quaternion *res);
void quat_product(struct quaternion *q1, struct quaternion *q2, struct quaternion *res);
void quat_inverse(struct quaternion *q, struct quaternion *res);
void euler_2_quat(float roll, float pitch, float yaw, struct quaternion *q);
void quat_2_dcm(struct quaternion *q, float mat[3][3]);
void quat_rotate_vector(struct quaternion *q, struct quaternion *vector, struct quaternion *res);
void quat_rotate_vector_inverse(struct quaternion *q, struct quaternion *vector, struct quaternion *res);
#endif