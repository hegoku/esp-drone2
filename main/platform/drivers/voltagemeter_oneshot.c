#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include <string.h>
#include "sensors/voltagemeter.h"
#include "platform/gpio_config/gpio_config.h"

#define READ_LEN 4

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

void init_voltagemeter()
{
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_12,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VOLTAGE_ADC_CHANNEL, &config));

	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};
	adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
}

void voltagemeter_read(float *output)
{
	int tmp_v = 0;
	int adc_raw;
	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, VOLTAGE_ADC_CHANNEL, &adc_raw));
	adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &tmp_v);
	*output = (float)tmp_v / 1000.0 * 2;
}