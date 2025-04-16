#ifndef MATH_MATH_H
#define MATH_MATH_H

#define M_PIf       3.14159265358979323846f

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#define constrain(v, min_v, max_b) (min(max(v,min_v),max_b))

#define DEGREES_TO_RADIANS(x) ((x)*M_PIf/180.0f)
#define RADIANS_TO_DEGREES(x) ((x)/(M_PIf/180.f))

float invSqrt(float x);

static inline float invNormal(float mat[3])
{
	return invSqrt(mat[0]*mat[0] + mat[1]*mat[1] + mat[2]*mat[2]);
}
#endif