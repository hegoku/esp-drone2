#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "misc/util.h"
#include "bus/spi.h"
#include "drivers/barometer/spl06.h"
#include "flight/flight.h"

static void get_calib_param(struct bus_dev *dev, struct spl06_calib_param *param);
static int spl06_get_sensor_data(struct barometer_sensor *sensor);
static int parse_sensor_data(const unsigned char *reg_data, int *temperature, int *pressure);

static int32_t sign_extend(uint32_t value, unsigned char bits)
{
	uint32_t sign_bit = 1u << (bits - 1);
	uint32_t mask = (1u << bits) - 1u;

	value &= mask;

	return (int32_t)((value ^ sign_bit) - sign_bit);
}

static float spl06_compensate_T_float(int adc_T, float k_t, struct spl06_calib_param *calib_param)
{
	float t_sc = (float)adc_T / k_t;

	return ((float)calib_param->c0 * 0.5f) + ((float)calib_param->c1 * t_sc);
}

static float get_scale_factor(unsigned char oversampling)
{
	switch (oversampling) {
	case SPL06_PRS_OVERSAMPLING_1X:
		return 524288.0f;
	case SPL06_PRS_OVERSAMPLING_2X:
		return 1572864.0f;
	case SPL06_PRS_OVERSAMPLING_4X:
		return 3670016.0f;
	case SPL06_PRS_OVERSAMPLING_8X:
		return 7864320.0f;
	case SPL06_PRS_OVERSAMPLING_16X:
		return 253952.0f;
	case SPL06_PRS_OVERSAMPLING_32X:
		return 516096.0f;
	case SPL06_PRS_OVERSAMPLING_64X:
		return 1040384.0f;
	case SPL06_PRS_OVERSAMPLING_128X:
		return 2088960.0f;
	default:
		return 0.0f;
	}
}

static float spl06_compensate_P_float(int adc_P, int adc_T, float k_p, float k_t, struct spl06_calib_param *calib_param)
{
	float p_sc = (float)adc_P / k_p;
	float t_sc = (float)adc_T / k_t;
	float qua2 = (float)calib_param->c10 + p_sc * ((float)calib_param->c20 + p_sc * (float)calib_param->c30);
	float qua3 = t_sc * p_sc * ((float)calib_param->c11 + p_sc * (float)calib_param->c21);

	return (float)calib_param->c00 + p_sc * qua2 + t_sc * (float)calib_param->c01 + qua3;
}

static int spl06_read(struct bus_dev *dev, unsigned char reg_addr, unsigned char *data, int len)
{
	return spi_read(dev, reg_addr, data, len);
}

static int spl06_write_byte(struct bus_dev *dev, unsigned char reg_addr, unsigned char data)
{
	unsigned char tdata = data;

	return spi_write(dev, reg_addr & SPL06_SPI_WR_MASK, &tdata, 1);
}

static void get_calib_param(struct bus_dev *dev, struct spl06_calib_param *param)
{
	unsigned char temp[SPL06_CALIB_DATA_SIZE];

	spl06_read(dev, SPL06_REG_CALIB_COEFF, temp, SPL06_CALIB_DATA_SIZE);

	param->c0 = (short)sign_extend(((uint32_t)temp[0] << 4) | ((uint32_t)temp[1] >> 4), 12);
	param->c1 = (short)sign_extend((((uint32_t)temp[1] & 0x0F) << 8) | (uint32_t)temp[2], 12);
	param->c00 = (int)sign_extend(((uint32_t)temp[3] << 12) | ((uint32_t)temp[4] << 4) | ((uint32_t)temp[5] >> 4), 20);
	param->c10 = (int)sign_extend((((uint32_t)temp[5] & 0x0F) << 16) | ((uint32_t)temp[6] << 8) | (uint32_t)temp[7], 20);
	param->c01 = (short)sign_extend(((uint32_t)temp[8] << 8) | (uint32_t)temp[9], 16);
	param->c11 = (short)sign_extend(((uint32_t)temp[10] << 8) | (uint32_t)temp[11], 16);
	param->c20 = (short)sign_extend(((uint32_t)temp[12] << 8) | (uint32_t)temp[13], 16);
	param->c21 = (short)sign_extend(((uint32_t)temp[14] << 8) | (uint32_t)temp[15], 16);
	param->c30 = (short)sign_extend(((uint32_t)temp[16] << 8) | (uint32_t)temp[17], 16);
}

static int spl06_get_sensor_data(struct barometer_sensor *sensor)
{
	unsigned char status;
	unsigned char temp[6];
	int temperature_raw;
	int pressure_raw;
	struct spl06_baro_priv *priv = SPL06_GET_PRIV(sensor->priv);

	sensor->status &= ~BARO_STATUS_DTRY;

	spl06_read(sensor->dev, SPL06_REG_MEAS_CFG, &status, 1);

	if ((status & (SPL06_MEAS_CFG_TMP_RDY | SPL06_MEAS_CFG_PRS_RDY)) != (SPL06_MEAS_CFG_TMP_RDY | SPL06_MEAS_CFG_PRS_RDY)) {
		return 0;
	}

	spl06_read(sensor->dev, SPL06_REG_PRESSURE_MSB, temp, 6);

	if (parse_sensor_data(temp, &temperature_raw, &pressure_raw) != 0) {
		return -1;
	}

	sensor->temperature.raw = temperature_raw;
	sensor->pressure.raw = (unsigned int)pressure_raw;

	priv->compensated_temperature = spl06_compensate_T_float(temperature_raw, priv->temperature_scale_factor, &priv->calib_param);
	sensor->temperature.value = priv->compensated_temperature;

	priv->compensated_pressure = spl06_compensate_P_float(pressure_raw, temperature_raw, priv->pressure_scale_factor,
		priv->temperature_scale_factor, &priv->calib_param);
	sensor->pressure.value = priv->compensated_pressure;
	sensor->status |= BARO_STATUS_DTRY;

	return 0;
}

static int parse_sensor_data(const unsigned char *reg_data, int *temperature, int *pressure)
{
	uint32_t pressure_data;
	uint32_t temperature_data;

	pressure_data = ((uint32_t)reg_data[0] << 16) | ((uint32_t)reg_data[1] << 8) | (uint32_t)reg_data[2];
	temperature_data = ((uint32_t)reg_data[3] << 16) | ((uint32_t)reg_data[4] << 8) | (uint32_t)reg_data[5];

	*pressure = (int)sign_extend(pressure_data, 24);
	*temperature = (int)sign_extend(temperature_data, 24);

	return 0;
}

int spl06_prob(struct bus_dev *dev)
{
	unsigned char id;
	unsigned char meas_cfg;
	int tries = 10;
	struct spl06_baro_priv *priv;

	spl06_read(dev, SPL06_REG_ID, &id, 1);
	if (id != SPL06_CHIP_ID)
		return -1;

	spl06_write_byte(dev, SPL06_REG_RESET, SPL06_RESET_SOFT_RST);
	delay_ms(10);

	while (tries--) {
		spl06_read(dev, SPL06_REG_MEAS_CFG, &meas_cfg, 1);
		if ((meas_cfg & (SPL06_MEAS_CFG_COEF_RDY | SPL06_MEAS_CFG_SENSOR_RDY)) == (SPL06_MEAS_CFG_COEF_RDY | SPL06_MEAS_CFG_SENSOR_RDY)) {
			break;
		}
		delay_ms(5);
	}

	if (tries < 0)
		return -1;

	dev->name = "SPL06";
	flight.baro.name = "SPL06";
	flight.baro.dev = dev;
	flight.baro.status = BARO_STATUS_ON;
	flight.baro.read = spl06_get_sensor_data;
	flight.baro.priv = malloc(sizeof(struct spl06_baro_priv));
	memset(flight.baro.priv, 0, sizeof(struct spl06_baro_priv));

	priv = SPL06_GET_PRIV(flight.baro.priv);
	get_calib_param(dev, &priv->calib_param);
	priv->pressure_scale_factor = get_scale_factor((SPL06_PRS_RATE_16HZ | SPL06_PRS_OVERSAMPLING_16X) & 0x07);
	priv->temperature_scale_factor = get_scale_factor((SPL06_TMP_EXT_SENSOR | SPL06_TMP_RATE_16HZ | SPL06_TMP_OVERSAMPLING_1X) & 0x07);

	spl06_write_byte(dev, SPL06_REG_PRS_CFG, SPL06_PRS_RATE_16HZ | SPL06_PRS_OVERSAMPLING_16X);
	spl06_write_byte(dev, SPL06_REG_TMP_CFG, SPL06_TMP_EXT_SENSOR | SPL06_TMP_RATE_16HZ | SPL06_TMP_OVERSAMPLING_1X);
	spl06_write_byte(dev, SPL06_REG_CFG_REG, SPL06_CFG_P_SHIFT);
	spl06_write_byte(dev, SPL06_REG_MEAS_CFG, SPL06_MEAS_CTRL_CONTINUOUS_BOTH);

	return 0;
}
