#include <nvs_flash.h>
#include <nvs.h>
#include "misc/config.h"

#define NVS_NAMESPACE "drone"
static nvs_handle_t my_handle;
static unsigned char is_in_transaction;

void init_config()
{
	esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
	}
    is_in_transaction = 0;
}

void config_begin_transaction()
{
    is_in_transaction = 1;
}

void config_commit()
{
    if (is_in_transaction==1) {
        nvs_commit(my_handle);
        is_in_transaction = 0;
    }
}

void config_write_uchar(char *key, unsigned char value)
{
    nvs_set_u8(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_char(char *key, char value)
{
    nvs_set_i8(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_ushort(char *key, unsigned short value)
{
    nvs_set_u16(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_short(char *key, short value)
{
    nvs_set_i16(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_uint(char *key, unsigned int value)
{
    nvs_set_u32(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_int(char *key, int value)
{
    nvs_set_i32(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_ulong(char *key, unsigned long long value)
{
    nvs_set_u64(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_long(char *key, long long value)
{
    nvs_set_i64(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_float(char *key, float value)
{
    nvs_set_u32(my_handle, key, *((unsigned int*)(&value)));
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_double(char *key, double value)
{
    nvs_set_i64(my_handle, key, *((unsigned long long*)(&value)));
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_write_string(char *key, char *value)
{
    nvs_set_str(my_handle, key, value);
    if (is_in_transaction==0) {
        nvs_commit(my_handle);
    }
}

void config_read_uchar(char *key, unsigned char *res)
{
    nvs_get_u8(my_handle, key, res);
}

void config_read_char(char *key, signed char *res)
{
    nvs_get_i8(my_handle, key, res);
}

void config_read_ushort(char *key, unsigned short *res)
{
    nvs_get_u16(my_handle, key, res);
}

void config_read_short(char *key, short *res)
{
    nvs_get_i16(my_handle, key, res);
}

void config_read_uint(char *key, long unsigned int *res)
{
    nvs_get_u32(my_handle, key, res);
}

void config_read_int(char *key, long int *res)
{
    nvs_get_i32(my_handle, key, res);
}

void config_read_ulong(char *key, unsigned long long *res)
{
    nvs_get_u64(my_handle, key, res);
}

void config_read_long(char *key, long long *res)
{
    nvs_get_i64(my_handle, key, res);
}

void config_read_float(char *key, float *res)
{
    nvs_get_u32(my_handle, key, ((long unsigned int*)res));
}

void config_read_double(char *key, double *res)
{
    nvs_get_u64(my_handle, key, (unsigned long long*)res);
}

void config_read_string(char *key, char *res)
{
    size_t size = 0;
    nvs_get_str(my_handle, key, NULL, &size);
    nvs_get_str(my_handle, key, res, &size);
}