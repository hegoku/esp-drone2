#ifndef MISC_IIR_FILTER_H
#define MISC_IIR_FILTER_H

struct iir_filter_param
{
	short int freq;
	short int cut_off_freq;
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;

	float y0;
	float y1;
	float y2;
	float x1;
	float x2;
};

void iir_filter_init(struct iir_filter_param *p);
float iir_filter(struct iir_filter_param *p, float input);

#endif