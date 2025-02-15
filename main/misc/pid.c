#include <math.h>
#include "misc/pid.h"

void pid_reset(struct pid_data *tdata)
{
	tdata->prev_error = 0.0;
	tdata->value = 0.0;
	tdata->error_sum = 0.0;
	tdata->output = 0.0;
	tdata->desired = 0.0;
	tdata->output = 0.0;
	tdata->pprev_error = 0.0;
	tdata->pid_d = 0.0;
	tdata->P = 0.0;
	tdata->I = 0.0;
	tdata->D = 0.0;
	tdata->pprev_desired = 0.0;
	tdata->prev_desired = 0.0;

	iir_filter_init(&tdata->filter);
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

	tdata->pid_d = (error - tdata->prev_error) / tdata->dt;
	if (tdata->enableFilter==1) {
		tdata->pid_d = iir_filter(&tdata->filter, tdata->pid_d);
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
	tdata->output = tdata->P + tdata->I + tdata->D;
	
	tdata->pprev_error = tdata->prev_error;
	tdata->prev_error = error;
	tdata->pprev_desired = tdata->prev_desired;
	tdata->prev_desired = desired;
}