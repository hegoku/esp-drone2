#ifndef ANOTC_CONFIG_FRAME_H
#define ANOTC_CONFIG_FRAME_H

#define ANOTC_FRAME_CONFIG_CMD 0xE0
#define ANOTC_FRAME_CONFIG_READ_WRITE 0xE1
#define ANOTC_FRAME_CONFIG_INFO 0xE2
#define ANOTC_FRAME_DEVICE_INFO 0xE3

#define ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO 0x00
#define ANOTC_CONFIG_FRAME_CMD_READ_COUNT 0x01
#define ANOTC_CONFIG_FRAME_CMD_READ_VALUE 0x02
#define ANOTC_CONFIG_FRAME_CMD_READ_INFO 0x03
#define ANOTC_CONFIG_FRAME_CMD_0X10 0x10

#define ANOTC_CONFIG_FRAME_VAL_RESET_PARAM 0xAA
#define ANOTC_CONFIG_FRAME_VAL_SAVE_PARAM 0xAB

enum ANOTC_PAR_TYPE
{
	ANOTC_PAR_TYPE_UINT8,
	ANOTC_PAR_TYPE_INT8,
	ANOTC_PAR_TYPE_UINT16,
	ANOTC_PAR_TYPE_INT16,
	ANOTC_PAR_TYPE_UINT32,
	ANOTC_PAR_TYPE_INT32,
	ANOTC_PAR_TYPE_UINT64,
	ANOTC_PAR_TYPE_INT64,
	ANOTC_PAR_TYPE_FLOAT,
	ANOTC_PAR_TYPE_DOUBLE,
	ANOTC_PAR_TYPE_STRING
};

void anotc_config_frame_read_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac);
void anotc_config_frame_write_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac);
void anotc_send_config_count(unsigned short count);
void anotc_send_config_info(unsigned short par_id, enum ANOTC_PAR_TYPE par_type, char *par_name, char *par_info);
void anotc_send_device_info(unsigned char dev_id, short hw_ver, short sw_ver, short bl_ver, short pt_ver, char *dev_name);
void anotc_send_config_value(unsigned short par_id, enum ANOTC_PAR_TYPE type, void *value);
#endif