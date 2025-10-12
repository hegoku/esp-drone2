#ifndef MISC_LOW_PASS_FILTER_2P_H
#define MISC_LOW_PASS_FILTER_2P_H

struct low_pass_filter_2p_param
{
	unsigned short freq;
	unsigned short cut_off_freq;
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;

	float x1;
	float x2;
};

void low_pass_filter_2p_init(struct low_pass_filter_2p_param *p);
float low_pass_filter_2p(struct low_pass_filter_2p_param *p, float input);

#endif