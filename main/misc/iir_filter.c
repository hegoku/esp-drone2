#include <math.h>
#include "misc/iir_filter.h"

void iir_filter_init(struct iir_filter_param *p)
{
	float fr = (float)p->freq/(float)p->cut_off_freq;
	float ohm = tanf(M_PI/fr);
	float c = 1.0+2.0*cosf(M_PI/4.0)*ohm+ohm*ohm;
	p->b0 = ohm*ohm/c;
	p->b1 = 2.0*p->b0;
	p->b2 = p->b0;
	p->a1 = 2.0*(ohm*ohm-1.0)/c;
	p->a2 = (1.0-2*cosf(M_PI/4.0)*ohm+ohm*ohm)/c;

	p->y0 = 0.0;
	p->y1 = 0.0;
	p->y2 = 0.0;
	p->x1 = 0.0;
	p->x2 = 0.0;
}

float iir_filter(struct iir_filter_param *p, float input)
{
	float x0 = input;
	p->y0 = (x0 * p->b0 + p->x1*p->b1 + p->x2*p->b2 - p->y0*p->a1 - p->y1*p->a2);
	p->y2 = p->y1;
	p->y1 = p->y0;
	p->x2 = p->x1;
	p->x1 = x0;
	return p->y2;
}
