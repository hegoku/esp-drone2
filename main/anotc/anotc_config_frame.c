#include "anotc/anotc.h"
#include "anotc/anotc_config_frame.h"

#define ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO 0x00
#define ANOTC_CONFIG_FRAME_CMD_READ_COUNT 0x01
#define ANOTC_CONFIG_FRAME_CMD_READ_VALUE 0x02
#define ANOTC_CONFIG_FRAME_CMD_READ_INFO 0x03
#define ANOTC_CONFIG_FRAME_CMD_0X10 0x10

void anotc_config_frame_cmd_handler(unsigned char *data)
{
	unsigned char cmd = data[0];
	unsigned short val = (data[2]<<8) | data[1];
	switch (cmd)
	{
	case ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO:
		anotc_send_device_info(ANOTC_DEVICE_ID, 1, 2, 0, 8, "esp32-drone");
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_COUNT:
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_VALUE:
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_INFO:
		break;
	default:
		break;
	}
}

void anotc_send_config_count(unsigned short count)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CONFIG_CMD;

	frame.data[frame.len++] = ANOTC_CONFIG_FRAME_CMD_READ_COUNT;
	anotc_add_ushort(&frame, count);
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_config_info(unsigned short par_id, enum ANOTC_PAR_TYPE par_type, char *par_name, char *par_info)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CONFIG_INFO;

	anotc_add_ushort(&frame, par_id);
	frame.data[frame.len++] = par_type;
	strncpy((char *)&(frame.data[frame.len]), par_name, 20);
	frame.len += 20;
	anotc_add_string(&frame, par_info);
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_device_info(unsigned char dev_id, short hw_ver, short sw_ver, short bl_ver, short pt_ver, char *dev_name)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_DEVICE_INFO;

	frame.data[frame.len++] = dev_id;
	anotc_add_short(&frame, hw_ver);
	anotc_add_short(&frame, sw_ver);
	anotc_add_short(&frame, bl_ver);
	anotc_add_short(&frame, pt_ver);
	anotc_add_string(&frame, dev_name);
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}
