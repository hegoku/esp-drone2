#include <math.h>
#include "math/math.h"
#include "misc/notch_filter.h"

void notch_filter_init(struct notch_filter_param *p)
{
	p->x1 = 0.0f;
    p->x2 = 0.0f;
    p->y1 = 0.0f;
    p->y2 = 0.0f;

    if (p->notch_freq <= 0.0f || p->freq <= 0 || p->bandwidth <= 0.0f) {
        p->b0 = 1.0f;
        p->b1 = 0.0f;
        p->b2 = 0.0f;
        p->a1 = 0.0f;
        p->a2 = 0.0f;
        return;
    }

    float omega = 2.0f * M_PIf * p->notch_freq / p->freq;
    float cs = cosf(omega);
    float alpha = tanf(M_PIf * p->bandwidth / p->freq);

    float a0_inv = 1.0f / (1.0f + alpha);

    p->b0 = a0_inv;
    p->b1 = -2.0f * cs * a0_inv;
    p->b2 = a0_inv;

    p->a1 = p->b1;
    p->a2 = (1.0f - alpha) * a0_inv;
}

float notch_filter(struct notch_filter_param *p, float input)
{
	float output = p->b0 * input
				+ p->b1 * p->x1
				+ p->b2 * p->x2
				- p->a1 * p->y1
				- p->a2 * p->y2;

    p->x2 = p->x1;
    p->x1 = input;

    p->y2 = p->y1;
    p->y1 = output;

    return output;
}