#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "misc/util.h"
#include "bus/spi.h"
#include "drivers/barometer/bmp280.h"
#include "flight/flight.h"

static int _st_check_boundaries(int utemperature, int upressure);
static int _parse_sensor_data(const unsigned char *reg_data, int *temperature, unsigned int *pressure);

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int bmp280_compensate_T_int32(int adc_T, int *t_fine, struct bmp2_calib_param *calib_param)
{
	int var1, var2, T;
	var1 = ((((adc_T>>3) - ((int)calib_param->dig_t1<<1))) * ((int)calib_param->dig_t2)) >> 11;
	var2 = (((((adc_T>>4) - ((int)calib_param->dig_t1)) * ((adc_T>>4) - ((int)calib_param->dig_t1))) >> 12) *
	((int)calib_param->dig_t3)) >> 14;
	*t_fine = var1 + var2;
	T = (*t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
unsigned int bmp280_compensate_P_int64(int adc_P, int t_fine, struct bmp2_calib_param *calib_param)
{
	long long int var1, var2, p;
	var1 = ((long long int)t_fine) - 128000;
	var2 = var1 * var1 * (long long int)calib_param->dig_p6;
	var2 = var2 + ((var1*(long long int)calib_param->dig_p5)<<17);
	var2 = var2 + (((long long int)calib_param->dig_p4)<<35);
	var1 = ((var1 * var1 * (long long int)calib_param->dig_p3)>>8) + ((var1 * (long long int)calib_param->dig_p2)<<12);
	var1 = (((((long long int)1)<<47)+var1))*((long long int)calib_param->dig_p1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((long long int)calib_param->dig_p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((long long int)calib_param->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((long long int)calib_param->dig_p7)<<4);
	return (unsigned int)p;
}

int bmp280_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len) {
	return spi_read(dev, reg_addr, data, len);
}

int bmp280_write_byte(struct bus_dev *dev, unsigned char reg_addr, unsigned char data) {
	unsigned char tdata = data;
	return spi_write(dev, reg_addr, &tdata, 1);
}

static void get_calib_param(struct bus_dev *dev, struct bmp2_calib_param *param)
{
    unsigned char temp[BMP2_CALIB_DATA_SIZE];

    bmp280_read(dev, BMP2_REG_DIG_T1_LSB, temp, BMP2_CALIB_DATA_SIZE);

	param->dig_t1 = (unsigned short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_T1_MSB_POS], temp[BMP2_DIG_T1_LSB_POS]));
	param->dig_t2 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_T2_MSB_POS], temp[BMP2_DIG_T2_LSB_POS]));
	param->dig_t3 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_T3_MSB_POS], temp[BMP2_DIG_T3_LSB_POS]));
	param->dig_p1 = (unsigned short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P1_MSB_POS], temp[BMP2_DIG_P1_LSB_POS]));
	param->dig_p2 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P2_MSB_POS], temp[BMP2_DIG_P2_LSB_POS]));
	param->dig_p3 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P3_MSB_POS], temp[BMP2_DIG_P3_LSB_POS]));
	param->dig_p4 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P4_MSB_POS], temp[BMP2_DIG_P4_LSB_POS]));
	param->dig_p5 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P5_MSB_POS], temp[BMP2_DIG_P5_LSB_POS]));
	param->dig_p6 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P6_MSB_POS], temp[BMP2_DIG_P6_LSB_POS]));
	param->dig_p7 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P7_MSB_POS], temp[BMP2_DIG_P7_LSB_POS]));
	param->dig_p8 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P8_MSB_POS], temp[BMP2_DIG_P8_LSB_POS]));
	param->dig_p9 = (short int) (BMP2_MSBLSB_TO_U16(temp[BMP2_DIG_P9_MSB_POS], temp[BMP2_DIG_P9_LSB_POS]));
	param->dig_p10 = (char) ((unsigned char)(temp[BMP2_DIG_P10_POS]));

	// printf("bmp280_calib_t1:%d %d %d\n", param->dig_t1, param->dig_t2, param->dig_t3);
	// printf("bmp280_calib_t1:%d %d %d %d %d %d %d %d %d %d\n", param->dig_p1, param->dig_p2, param->dig_p3
	// 	, param->dig_p4 , param->dig_p5 , param->dig_p6
	// 	, param->dig_p7 , param->dig_p8 , param->dig_p9
	// 	, param->dig_p10
	// );
}

int bmp280_get_sensor_data(struct barometer_sensor *sensor)
{
	unsigned char temp[6];
	struct bmp280_baro_priv *priv = BMP280_GET_PRIV(sensor->priv);

	bmp280_read(sensor->dev, BMP280_REG_PRESS_MSB, temp, 6);
	if (_parse_sensor_data(temp, &sensor->temperature.raw, &sensor->pressure.raw)==0) {
		priv->compensated_temperature = bmp280_compensate_T_int32(sensor->temperature.raw, &priv->t_fine, &priv->calib_param);
		sensor->temperature.value = ((float)priv->compensated_temperature) / 100.0;
		priv->compensated_pressure = bmp280_compensate_P_int64(sensor->pressure.raw, priv->t_fine, &priv->calib_param);
		sensor->pressure.value = ((float)priv->compensated_pressure) / 256.0;
		// bmp280_data.altitude = 44330.0 * (1.0-pow(bmp280_data.pressure/100.0/1018.7, 1.0f/5.255f)) * 100.0;
	// sensor->altitude = 44330.0 * (1.0-pow(sensor->pressure.value/100.0/1013.25, 1.0f/5.255f)) * 100.0;
		// bmp280_data.altitude = ((powf(101325.0 / bmp280_data.pressure, 1.0 / 5.257) - 1) * (bmp280_data.temperature + 273.15)) / 0.000065;
		return 0;
	}
	else
	{
		return -1;
	}
}

/*!
 *  @brief This internal API is used to parse the pressure and temperature
 *  data and store it in the bmp2_uncomp_data structure instance.
 */
static int _parse_sensor_data(const unsigned char *reg_data, int *temperature, unsigned int *pressure)
{
    int rslt;

    /* Variables to store the sensor data */
    unsigned int data_xlsb;
    unsigned int data_lsb;
    unsigned int data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (unsigned int)reg_data[0] << 12;
    data_lsb = (unsigned int)reg_data[1] << 4;
    data_xlsb = (unsigned int)reg_data[2] >> 4;
    *pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (int)reg_data[3] << 12;
    data_lsb = (int)reg_data[4] << 4;
    data_xlsb = (int)reg_data[5] >> 4;
    *temperature = (int)(data_msb | data_lsb | data_xlsb);

    rslt = _st_check_boundaries((int)*temperature, (int)*pressure);

    return rslt;
}

/*!
 * @This internal API checks whether the uncompensated temperature and
 * uncompensated pressure are within the range
 */
static int _st_check_boundaries(int utemperature, int upressure)
{
    int rslt;

    /* Check Uncompensated pressure in not valid range AND uncompensated temperature in valid range */
    if ((upressure < BMP2_ST_ADC_P_MIN || upressure > BMP2_ST_ADC_P_MAX) &&
        (utemperature >= BMP2_ST_ADC_T_MIN && utemperature <= BMP2_ST_ADC_T_MAX))
    {
        rslt = BMP2_E_UNCOMP_PRESS_RANGE;
    }
    /* Check Uncompensated temperature in not valid range AND uncompensated pressure in valid range */
    else if ((utemperature < BMP2_ST_ADC_T_MIN || utemperature > BMP2_ST_ADC_T_MAX) &&
             (upressure >= BMP2_ST_ADC_P_MIN && upressure <= BMP2_ST_ADC_P_MAX))
    {
        rslt = BMP2_E_UNCOMP_TEMP_RANGE;
    }
    /* Check Uncompensated pressure in not valid range AND uncompensated temperature in not valid range */
    else if ((upressure < BMP2_ST_ADC_P_MIN || upressure > BMP2_ST_ADC_P_MAX) &&
             (utemperature < BMP2_ST_ADC_T_MIN || utemperature > BMP2_ST_ADC_T_MAX))
    {
        rslt = BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE;
    }
    else
    {
        rslt = 0;
    }

    return rslt;
}

int bmp280_prob(struct bus_dev *dev)
{
	unsigned char id;
	bmp280_read(dev, BMP280_REG_ID, &id, 1);
	if (id!=BMP280_CHIP_ID)
		return -1;
	bmp280_write_byte(dev, BMP280_REG_RESET ^ BMP2_SPI_RD_MASK, 0xB6); // reset
	delay_ms(10);

	dev->name = "BMP280";
	flight.baro.name = "BMP280";
	flight.baro.dev = dev;
	flight.baro.status = BARO_STATUS_ON;
	flight.baro.read = bmp280_get_sensor_data;
	flight.baro.priv = malloc(sizeof(struct bmp280_baro_priv));

	get_calib_param(dev, &(BMP280_GET_PRIV(flight.baro.priv)->calib_param));
	bmp280_write_byte(dev, BMP280_REG_CONFIG ^ BMP2_SPI_RD_MASK, (BMP280_STANDBY_0_5MS << 5) | (BMP2_FILTER_COEFF_16 << 2) | BMP2_SPI3_WIRE_DISABLE);
	bmp280_write_byte(dev, BMP280_REG_CTRL_MEAS ^ BMP2_SPI_RD_MASK, (BMP280_OSRS_2X << 5) | (BMP280_OSRS_16X << 2) | BMP280_MODE_SLEEP);
	return 0;
}