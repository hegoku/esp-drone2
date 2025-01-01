#ifndef DRIVERS_BARO_BMP280_H
#define DRIVERS_BARO_BMP280_H

#include "bus/bus.h"

#define BMP2_I2C_ADDRESS1 (0x3B<<1)
#define BMP2_I2C_ADDRESS2 (BMP2_I2C_ADDRESS1+1)

#define BMP2_REG_DIG_T1_LSB 0x88
#define BMP2_REG_DIG_T1_MSB 0x89
#define BMP2_REG_DIG_T2_LSB 0x8A
#define BMP2_REG_DIG_T2_MSB 0x8B
#define BMP2_REG_DIG_T3_LSB 0x8C
#define BMP2_REG_DIG_T3_MSB 0x8D
#define BMP2_REG_DIG_P1_LSB 0x8E
#define BMP2_REG_DIG_P1_MSB 0x8F
#define BMP2_REG_DIG_P2_LSB 0x90
#define BMP2_REG_DIG_P2_MSB 0x91
#define BMP2_REG_DIG_P3_LSB 0x92
#define BMP2_REG_DIG_P3_MSB 0x93
#define BMP2_REG_DIG_P4_LSB 0x94
#define BMP2_REG_DIG_P4_MSB 0x95
#define BMP2_REG_DIG_P5_LSB 0x96
#define BMP2_REG_DIG_P5_MSB 0x97
#define BMP2_REG_DIG_P6_LSB 0x98
#define BMP2_REG_DIG_P6_MSB 0x99
#define BMP2_REG_DIG_P7_LSB 0x9A
#define BMP2_REG_DIG_P7_MSB 0x9B
#define BMP2_REG_DIG_P8_LSB 0x9C
#define BMP2_REG_DIG_P8_MSB 0x9D
#define BMP2_REG_DIG_P9_LSB 0x9E
#define BMP2_REG_DIG_P9_MSB 0x9F

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_XLSB 0xFC

#define BMP280_OSRS_NONE 0x00
#define BMP280_OSRS_1X 0x01
#define BMP280_OSRS_2X 0x02
#define BMP280_OSRS_4X 0x03
#define BMP280_OSRS_8X 0x04
#define BMP280_OSRS_16X 0x05

#define BMP280_MODE_SLEEP 0x00
#define BMP280_MODE_FORCED0 0x01
#define BMP280_MODE_FORCED1 0x02
#define BMP280_MODE_NORMAL 0x03

#define BMP280_STANDBY_0_5MS 0x00
#define BMP280_STANDBY_62_5MS 0x01
#define BMP280_STANDBY_125MS 0x02
#define BMP280_STANDBY_250MS 0x03
#define BMP280_STANDBY_500MS 0x04
#define BMP280_STANDBY_1000MS 0x05
#define BMP280_STANDBY_2000MS 0x06
#define BMP280_STANDBY_4000MS 0x07

#define BMP2_FILTER_OFF 0x00
#define BMP2_FILTER_COEFF_2 0x01
#define BMP2_FILTER_COEFF_4 0x02
#define BMP2_FILTER_COEFF_8 0x03
#define BMP2_FILTER_COEFF_16 0x04

#define BMP2_SPI3_WIRE_ENABLE 0x01
#define BMP2_SPI3_WIRE_DISABLE 0x00

#define BMP2_SPI_RD_MASK 0x80
#define BMP2_SPI_WR_MASK 0x7F

/*! @name Calibration parameters' relative position */
#define BMP2_DIG_T1_LSB_POS 0x00
#define BMP2_DIG_T1_MSB_POS 0x01
#define BMP2_DIG_T2_LSB_POS 0x02
#define BMP2_DIG_T2_MSB_POS 0x03
#define BMP2_DIG_T3_LSB_POS 0x04
#define BMP2_DIG_T3_MSB_POS 0x05
#define BMP2_DIG_P1_LSB_POS 0x06
#define BMP2_DIG_P1_MSB_POS 0x07
#define BMP2_DIG_P2_LSB_POS 0x08
#define BMP2_DIG_P2_MSB_POS 0x09
#define BMP2_DIG_P3_LSB_POS 0x0A
#define BMP2_DIG_P3_MSB_POS 0x0B
#define BMP2_DIG_P4_LSB_POS 0x0C
#define BMP2_DIG_P4_MSB_POS 0x0D
#define BMP2_DIG_P5_LSB_POS 0x0E
#define BMP2_DIG_P5_MSB_POS 0x0F
#define BMP2_DIG_P6_LSB_POS 0x10
#define BMP2_DIG_P6_MSB_POS 0x11
#define BMP2_DIG_P7_LSB_POS 0x12
#define BMP2_DIG_P7_MSB_POS 0x13
#define BMP2_DIG_P8_LSB_POS 0x14
#define BMP2_DIG_P8_MSB_POS 0x15
#define BMP2_DIG_P9_LSB_POS 0x16
#define BMP2_DIG_P9_MSB_POS 0x17
#define BMP2_DIG_P10_POS    0x18
#define BMP2_CALIB_DATA_SIZE 0x19

/*! @name Error codes */
#define BMP2_E_NULL_PTR -1
#define BMP2_E_COM_FAIL -2
#define BMP2_E_INVALID_LEN -3
#define BMP2_E_DEV_NOT_FOUND -4
#define BMP2_E_UNCOMP_TEMP_RANGE -5
#define BMP2_E_UNCOMP_PRESS_RANGE -6
#define BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE -7

/* 0x00000 is minimum output value */
#define BMP2_ST_ADC_T_MIN 0x00000

/* 0xFFFF0 is maximum 20-bit output value without over sampling */
#define BMP2_ST_ADC_T_MAX 0xFFFF0

/* 0x00000 is minimum output value */
#define BMP2_ST_ADC_P_MIN 0x00000

/* 0xFFFF0 is maximum 20-bit output value without over sampling */
#define BMP2_ST_ADC_P_MAX 0xFFFF0

#define BMP280_CHIP_ID 0x58

#define BMP2_MSBLSB_TO_U16(msb, lsb) (((unsigned short)msb << 8) | ((unsigned short)lsb))

struct bmp2_calib_param
{
    /*! Calibration parameter of temperature data */

    /*! Calibration t1 data */
    unsigned short int dig_t1;

    /*! Calibration t2 data */
    short int dig_t2;

    /*! Calibration t3 data */
    short int dig_t3;

    /*! Calibration parameter of pressure data */

    /*! Calibration p1 data */
    unsigned short int dig_p1;

    /*! Calibration p2 data */
    short int dig_p2;

    /*! Calibration p3 data */
    short int dig_p3;

    /*! Calibration p4 data */
    short int dig_p4;

    /*! Calibration p5 data */
    short int dig_p5;

    /*! Calibration p6 data */
    short int dig_p6;

    /*! Calibration p7 data */
    short int dig_p7;

    /*! Calibration p8 data */
    short int dig_p8;

    /*! Calibration p9 data */
    short int dig_p9;

    /*! Calibration p10 data */
    char dig_p10;

    /*! Fine resolution temperature value */
    int t_fine;
};

struct bmp280_baro_priv {
	//t_fine carries fine temperature as global value
	int t_fine;
	int compensated_temperature;
	unsigned int compensated_pressure;

	struct bmp2_calib_param calib_param;
};

#define BMP280_GET_PRIV(x) ((struct bmp280_baro_priv*)x)

int bmp280_prob(struct bus_dev *dev);
#endif