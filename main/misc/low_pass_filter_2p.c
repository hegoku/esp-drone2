#include <math.h>
#include "math/math.h"
#include "misc/low_pass_filter_2p.h"

void low_pass_filter_2p_init(struct low_pass_filter_2p_param *p)
{
    p->x1 = 0.0f;
    p->x2 = 0.0f;

    if (p->cut_off_freq <= 0 || p->freq <= 0) {
        p->b0 = 1.0f;
        p->b1 = 0.0f;
        p->b2 = 0.0f;
        p->a1 = 0.0f;
        p->a2 = 0.0f;
        return;
    }

	float fr = (float)p->freq/(float)p->cut_off_freq;
	float ohm = tanf(M_PIf/fr);
	float c = 1.0f+2.0f*cosf(M_PIf/4.0f)*ohm+ohm*ohm;
	p->b0 = ohm*ohm/c;
	p->b1 = 2.0f*p->b0;
	p->b2 = p->b0;
	p->a1 = 2.0f*(ohm*ohm-1.0f)/c;
	p->a2 = (1.0f-2.0f*cosf(M_PIf/4.0f)*ohm+ohm*ohm)/c;
}

float low_pass_filter_2p(struct low_pass_filter_2p_param *p, float input)
{
	float x0 = input - p->a1 * p->x1 - p->a2 * p->x2;
    float y  = p->b0 * x0 + p->b1 * p->x1 + p->b2 * p->x2;

    p->x2 = p->x1;
    p->x1 = x0;

    return y;
}
