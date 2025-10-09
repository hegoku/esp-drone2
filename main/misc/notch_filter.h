#ifndef MISC_NOTCH_FILTER_H
#define MISC_NOTCH_FILTER_H

struct notch_filter_param
{
	float freq;
	float notch_freq;             // 陷波频率 Hz
    float bandwidth;              // 陷波带宽 Hz
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;

	float x1;
    float x2;
    float y1;
    float y2;
};

void notch_filter_init(struct notch_filter_param *p);
float notch_filter(struct notch_filter_param *p, float input);

#endif