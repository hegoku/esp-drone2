#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include <string.h>
#include "sensors/voltagemeter.h"
#include "platform/gpio_config/gpio_config.h"

#define READ_LEN 4

adc_continuous_handle_t handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;

void init_voltagemeter()
{
	adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 256,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

	adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
		.pattern_num = 1
    };

	adc_digi_pattern_config_t adc_pattern[1] = {0};
	adc_pattern[0].atten = ADC_ATTEN_DB_12;
	adc_pattern[0].channel = VOLTAGE_ADC_CHANNEL & 0x7;
	adc_pattern[0].unit = ADC_UNIT_1;
	adc_pattern[0].bit_width = ADC_BITWIDTH_9;
	dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

	adc_cali_line_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_9,
	};
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle));

    ESP_ERROR_CHECK(adc_continuous_start(handle));
}

void voltagemeter_read(float *output)
{
	int voltage = 0;
	uint32_t ret_num = 0;
	uint8_t result[READ_LEN] = {0};
	memset(result, 0xcc, READ_LEN);
	int ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
	if (ret==ESP_OK) {
		for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
			adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
			uint32_t chan_num = p->type1.channel;
			uint32_t data = p->type1.data;
			/* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
			if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
				ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, data, &voltage));
				*output = (float)voltage / 1000.0 * 2;
			}
		}
	}
}