#include <math.h>
#include "controllers/pid.h"

void pid_init(struct pid_data *tdata)
{
	pid_reset(tdata);
	low_pass_filter_2p_init(&tdata->filter);
}

void pid_reset(struct pid_data *tdata)
{
	tdata->prev_error = 0.0f;
	tdata->value = 0.0f;
	tdata->error_sum = 0.0f;
	tdata->output = 0.0f;
	tdata->desired = 0.0f;
	tdata->output = 0.0f;
	tdata->pprev_error = 0.0f;
	tdata->pid_d = 0.0f;
	tdata->P = 0.0f;
	tdata->I = 0.0f;
	tdata->D = 0.0f;
	tdata->pprev_desired = 0.0f;
	tdata->prev_desired = 0.0f;
	tdata->prev_value = 0.0f;

	low_pass_filter_2p_init(&tdata->filter);
}

void pid_calculate(struct pid_data *tdata, float desired)
{
	float error = 0.0f;

	tdata->desired = desired;
	error = desired - tdata->value;
	if (error>0 && tdata->error_sum>0) {

	} else if (error<=0 && tdata->error_sum<0) {

	} else {
		tdata->error_sum = 0;
	}

	tdata->pid_d = (tdata->value - tdata->prev_value) / tdata->dt;
	if (tdata->enable_filter==1) {
		tdata->pid_d = low_pass_filter_2p(&tdata->filter, tdata->pid_d);
	}
	// if (isnan(tdata->pid_d)) {
	// 	tdata->pid_d = 0.0f; 
	// }

	if (tdata->error_sum+error * tdata->dt>tdata->err_limit) {
		tdata->error_sum = tdata->err_limit;
	} else if (tdata->error_sum+error * tdata->dt<-tdata->err_limit) {
		tdata->error_sum = -tdata->err_limit;
	} else {
		tdata->error_sum += error * tdata->dt;
	}
	tdata->P = tdata->kp * error;
	tdata->I = tdata->ki * tdata->error_sum;
	tdata->D = tdata->kd * tdata->pid_d;
	// if (tdata->D>10) {
	// 	tdata->D = 10;
	// }
	// if (tdata->D<-10) {
	// 	tdata->D = -10;
	// }
	tdata->output = tdata->P + tdata->I - tdata->D + tdata->ff*tdata->desired;
	
	tdata->pprev_error = tdata->prev_error;
	tdata->prev_error = error;
	tdata->pprev_desired = tdata->prev_desired;
	tdata->prev_desired = desired;
	tdata->prev_value = tdata->value;
}