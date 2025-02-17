#include "sensors/imu.h"
#include "flight/flight.h"
#include "misc/iir_filter.h"
#include "clocksource/imu_source.h"
#include "misc/config.h"
#include "anotc/anotc_cmd_frame.h"
#include "math/matrix.h"
#include "misc/geo.h"

#define DIRECTION_UP 0
#define DIRECTION_DOWN 1
#define DIRECTION_FORWARD 2
#define DIRECTION_BACKWARD 3
#define DIRECTION_LEFT 4
#define DIRECTION_RIGHT 5

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

static struct iir_filter_param acc_iir[3];
static struct iir_filter_param gyr_iir[3];

struct gyro_calibration {
	float x;
	float y;
	float z;
	unsigned int count;
	unsigned char percentage;
};

struct accel_calibration {
	float ref_mat[DIRECTION_RIGHT+1][3];
	float x_avg;
	float y_avg;
	float z_avg;
	unsigned int count;
	unsigned char done_mask;
	unsigned char percentage;
};

static struct gyro_calibration gyro_c;
static struct accel_calibration accel_c;

void init_imu(struct imu_sensor *sensor)
{
	sensor->accel.calibration.x_k = 1.0;
	sensor->accel.calibration.y_k = 1.0;
	sensor->accel.calibration.z_k = 1.0;

	config_read_float("accel_k.x", &(sensor->accel.calibration.x_k));
	config_read_float("accel_k.y", &(sensor->accel.calibration.y_k));
	config_read_float("accel_k.z", &(sensor->accel.calibration.z_k));
	
	config_read_float("accel_offset.x", &(sensor->accel.calibration.x_offset));
	config_read_float("accel_offset.y", &(sensor->accel.calibration.y_offset));
	config_read_float("accel_offset.z", &(sensor->accel.calibration.z_offset));

	config_read_float("gyro_offset.x", &(sensor->gyro.calibration.x_offset));
	config_read_float("gyro_offset.y", &(sensor->gyro.calibration.y_offset));
	config_read_float("gyro_offset.z", &(sensor->gyro.calibration.z_offset));

	if (sensor->status != IMU_STATUS_ON)
		return;

	for (int i = 0; i < 3; i++)
	{
		acc_iir[i].cut_off_freq = 10;
		acc_iir[i].freq = sensor->freq;
		iir_filter_init(&acc_iir[i]);

		gyr_iir[i].cut_off_freq = 10;
		gyr_iir[i].freq = sensor->freq;
		iir_filter_init(&gyr_iir[i]);
	}

	imu_timer.freq = sensor->freq;
	sys_timer_set(&imu_timer);
}

void imu_filter(struct imu_sensor *sensor)
{
	sensor->accel.unfiltered.x = sensor->accel.calibration.x_k * (sensor->accel.unfiltered.x - sensor->accel.calibration.x_offset);
	sensor->accel.unfiltered.y = sensor->accel.calibration.y_k * (sensor->accel.unfiltered.y - sensor->accel.calibration.y_offset);
	sensor->accel.unfiltered.z = sensor->accel.calibration.z_k * (sensor->accel.unfiltered.z - sensor->accel.calibration.z_offset);
	sensor->accel.value.x = iir_filter(&acc_iir[0], sensor->accel.unfiltered.x);
	sensor->accel.value.y = iir_filter(&acc_iir[1], sensor->accel.unfiltered.y);
	sensor->accel.value.z = iir_filter(&acc_iir[2], sensor->accel.unfiltered.z);

	sensor->gyro.unfiltered.x -=  sensor->gyro.calibration.x_offset;
	sensor->gyro.unfiltered.y -=  sensor->gyro.calibration.y_offset;
	sensor->gyro.unfiltered.z -=  sensor->gyro.calibration.z_offset;
	sensor->gyro.value.x = iir_filter(&gyr_iir[0], sensor->gyro.unfiltered.x);
	sensor->gyro.value.y = iir_filter(&gyr_iir[1], sensor->gyro.unfiltered.y);
	sensor->gyro.value.z = iir_filter(&gyr_iir[2], sensor->gyro.unfiltered.z);
}

void calibrate_accel(struct imu_sensor *sensor)
{
	float mat_A[3][3];
	float mat_A_inverse[3][3] = {{.0f,.0f,.0f}, {.0f,.0f,.0f}, {.0f,.0f,.0f}};
	float Accel_T[3][3] = {{.0f,.0f,.0f}, {.0f,.0f,.0f}, {.0f,.0f,.0f}};
	float mat_U[3][3] = {{CONSTANTS_ONE_G, .0f, .0f}, {.0f, CONSTANTS_ONE_G, .0f}, {.0f, .0f, CONSTANTS_ONE_G}};

	//calculate offset
	sensor->accel.calibration.x_offset = (accel_c.ref_mat[DIRECTION_FORWARD][AXIS_X] + accel_c.ref_mat[DIRECTION_BACKWARD][AXIS_X]) * 0.5f;
	sensor->accel.calibration.y_offset = (accel_c.ref_mat[DIRECTION_LEFT][AXIS_Y] + accel_c.ref_mat[DIRECTION_RIGHT][AXIS_Y]) * 0.5f;
	sensor->accel.calibration.z_offset = (accel_c.ref_mat[DIRECTION_UP][AXIS_Z] + accel_c.ref_mat[DIRECTION_DOWN][AXIS_Z]) * 0.5f;

	// x
	mat_A[0][0] = accel_c.ref_mat[DIRECTION_BACKWARD][AXIS_X] - sensor->accel.calibration.x_offset;
	mat_A[0][1] = accel_c.ref_mat[DIRECTION_BACKWARD][AXIS_Y] - sensor->accel.calibration.y_offset;
	mat_A[0][2] = accel_c.ref_mat[DIRECTION_BACKWARD][AXIS_Z] - sensor->accel.calibration.z_offset;

	//y
	mat_A[1][0] = accel_c.ref_mat[DIRECTION_RIGHT][AXIS_X] - sensor->accel.calibration.x_offset;
	mat_A[1][1] = accel_c.ref_mat[DIRECTION_RIGHT][AXIS_Y] - sensor->accel.calibration.y_offset;
	mat_A[1][2] = accel_c.ref_mat[DIRECTION_RIGHT][AXIS_Z] - sensor->accel.calibration.z_offset;

	//z
	mat_A[2][0] = accel_c.ref_mat[DIRECTION_UP][AXIS_X] - sensor->accel.calibration.x_offset;
	mat_A[2][1] = accel_c.ref_mat[DIRECTION_UP][AXIS_Y] - sensor->accel.calibration.y_offset;
	mat_A[2][2] = accel_c.ref_mat[DIRECTION_UP][AXIS_Z] - sensor->accel.calibration.z_offset;

	matrix_inverse(mat_A, 3, mat_A_inverse);
	matrix_mult(mat_A_inverse, mat_U, Accel_T);

	sensor->accel.calibration.x_k = Accel_T[0][0];
	sensor->accel.calibration.y_k = Accel_T[1][1];
	sensor->accel.calibration.z_k = Accel_T[2][2];

	config_write_float("accel_k.x", sensor->accel.calibration.x_k);
	config_write_float("accel_k.y", sensor->accel.calibration.y_k);
	config_write_float("accel_k.z", sensor->accel.calibration.z_k);
	
	config_write_float("accel_offset.x", sensor->accel.calibration.x_offset);
	config_write_float("accel_offset.y", sensor->accel.calibration.y_offset);
	config_write_float("accel_offset.z", sensor->accel.calibration.z_offset);

	config_write_float("gyro_offset.x", sensor->gyro.calibration.x_offset);
	config_write_float("gyro_offset.y", sensor->gyro.calibration.y_offset);
	config_write_float("gyro_offset.z", sensor->gyro.calibration.z_offset);
}

static void _calibrate_accel(struct imu_sensor *sensor, unsigned char direction, float direction_value[3], unsigned char mask)
{
	unsigned char d[2] = {direction, 0};
	accel_c.x_avg += sensor->accel.unfiltered.x;
	accel_c.y_avg += sensor->accel.unfiltered.y;
	accel_c.z_avg += sensor->accel.unfiltered.z;
	accel_c.count++;
	accel_c.percentage = (unsigned char)(((float)accel_c.count) / 10000.0f * 100.0f);
	if (accel_c.percentage%20==0) {
		d[1] = accel_c.percentage;
		anotc_send_cmd_response(ANOTC_CMD_CALIBRATE_ACCEL, 0, d, sizeof(d));
	}
	if (accel_c.count>=10000) {
		direction_value[AXIS_X] = accel_c.x_avg / (float)accel_c.count;
		direction_value[AXIS_Y] = accel_c.y_avg / (float)accel_c.count;
		direction_value[AXIS_Z] = accel_c.z_avg / (float)accel_c.count;
		accel_c.x_avg = 0;
		accel_c.y_avg = 0;
		accel_c.z_avg = 0;
		flight.status = FLIGHT_STATUS_CALIBRATION_ACCEL;
		accel_c.count = 0;
		accel_c.done_mask |= mask;
		if (accel_c.done_mask == 0x3F) {
			calibrate_accel(sensor);
			d[0] = 0;
			flight.status = FLIGHT_STATUS_READY;
			anotc_send_cmd_response(ANOTC_CMD_CALIBRATE_ACCEL, 0, d, sizeof(d));
			accel_c.done_mask = 0;
		}
	}
}

void imu_calibration(struct imu_sensor *sensor)
{
	if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_UP) {
		_calibrate_accel(sensor, 'U', accel_c.ref_mat[DIRECTION_UP], 0x1);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_DOWN) {
		_calibrate_accel(sensor, 'D', accel_c.ref_mat[DIRECTION_DOWN], 0x2);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_FORWARD) {
		_calibrate_accel(sensor, 'F', accel_c.ref_mat[DIRECTION_FORWARD], 0x4);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_BACKWARD) {
		_calibrate_accel(sensor, 'B', accel_c.ref_mat[DIRECTION_BACKWARD], 0x8);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_LEFT) {
		_calibrate_accel(sensor, 'L', accel_c.ref_mat[DIRECTION_LEFT], 0x10);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_ACCEL_RIGHT) {
		_calibrate_accel(sensor, 'R', accel_c.ref_mat[DIRECTION_RIGHT], 0x20);
	} else if (flight.status==FLIGHT_STATUS_CALIBRATION_GYRO) {
		gyro_c.x += sensor->gyro.unfiltered.x;
		gyro_c.y += sensor->gyro.unfiltered.y;
		gyro_c.z += sensor->gyro.unfiltered.z;
		gyro_c.count++;

		gyro_c.percentage = (unsigned char)(((float)gyro_c.count) / 10000.0f * 100.0f);
		if (gyro_c.percentage%20==0) {
			anotc_send_cmd_response(ANOTC_CMD_CALIBRATE_GYRO, 0, &gyro_c.percentage, 1);
		}

		if (gyro_c.count>=10000) {
			gyro_c.x /= (float)gyro_c.count;
			gyro_c.y /= (float)gyro_c.count;
			gyro_c.z /= (float)gyro_c.count;

			sensor->gyro.calibration.x_offset = gyro_c.x;
			sensor->gyro.calibration.y_offset = gyro_c.y;
			sensor->gyro.calibration.z_offset = gyro_c.z;

			config_write_float("gyro_offset.x", sensor->gyro.calibration.x_offset);
			config_write_float("gyro_offset.y", sensor->gyro.calibration.y_offset);
			config_write_float("gyro_offset.z", sensor->gyro.calibration.z_offset);

			flight.status = FLIGHT_STATUS_READY;

			gyro_c.x = 0;
			gyro_c.y = 0;
			gyro_c.z = 0;
			gyro_c.count = 0;
		}
	}
}