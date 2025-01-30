#ifndef MISC_CONFIG_H
#define MISC_CONFIG_H

void config_wirte_uchar(char *key, unsigned char value);
void config_write_char(char *key, char value);
void config_write_ushort(char *key, unsigned short value);
void config_write_short(char *key, short value);
void config_write_uint(char *key, unsigned int value);
void config_write_int(char *key, int value);
void config_write_ulong(char *key, unsigned long long value);
void config_write_long(char *key, long long value);
void config_write_float(char *key, float value);
void config_write_double(char *key, double value);
void config_write_string(char *key, char *value);

void config_read_uchar(char *key, unsigned char *res);
void config_read_char(char *key, signed char *res);
void config_read_ushort(char *key, unsigned short *res);
void config_read_short(char *key, short *res);
void config_read_uint(char *key, long unsigned int *res);
void config_read_int(char *key, long int *res);
void config_read_ulong(char *key, unsigned long long *res);
void config_read_long(char *key, long long *res);
void config_read_float(char *key, float *res);
void config_read_double(char *key, double *res);
void config_read_string(char *key, char *res);

void config_begin_transaction();
void config_commit();

void init_config();
#endif