#ifndef MISC_PID_H
#define MISC_PID_H

#include "misc/iir_filter.h"

struct pid_data {
	float kp;
	float ki;
	float kd;
	float prev_error;
	float value;
	float dt;
	float error_sum;
	float output;
	float err_limit;
	char enable_filter;
	float output_limit;
	struct iir_filter_param filter;

	float P;
	float I;
	float D;

	float desired;
	float pid_d;
	float pprev_error;
	float prev_desired;
	float pprev_desired;
	float prev_value;
};

void pid_init(struct pid_data *tdata);
void pid_reset(struct pid_data *tdata);
void pid_calculate(struct pid_data *tdata, float desired);

#endif