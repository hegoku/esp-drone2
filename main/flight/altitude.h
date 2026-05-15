#ifndef FLIGHT_ALTITUDE_H
#define FLIGHT_ALTITUDE_H

void calculate_altitude();
float altitude_get_gravity();
float altitude_get_accel_bias();
float altitude_get_baro_bias();
float altitude_get_k_altitude();
float altitude_get_k_velocity();
float altitude_get_k_accel_bias();
float altitude_get_k_baro_bias();

#endif
