#include "anotc/anotc.h"
#include "anotc/anotc_config_frame.h"

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

void anotc_send_config_value(unsigned short par_id, enum ANOTC_PAR_TYPE type, void *value)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CONFIG_READ_WRITE;

	anotc_add_ushort(&frame, par_id);
	switch (type)
	{
	case ANOTC_PAR_TYPE_UINT8:
		frame.data[frame.len++] = *((unsigned char*)value);
		break;
	case ANOTC_PAR_TYPE_INT8:
		frame.data[frame.len++] = *((char*)value);
		break;
	case ANOTC_PAR_TYPE_UINT16:
		anotc_add_ushort(&frame, *((unsigned short*)value));
		break;
	case ANOTC_PAR_TYPE_INT16:
		anotc_add_short(&frame, *((short*)value));
		break;
	case ANOTC_PAR_TYPE_UINT32:
		anotc_add_uint(&frame, *((unsigned int*)value));
		break;
	case ANOTC_PAR_TYPE_INT32:
		anotc_add_int(&frame, *((int*)value));
		break;
	case ANOTC_PAR_TYPE_UINT64:
		anotc_add_ulong(&frame, *((unsigned long long*)value));
		break;
	case ANOTC_PAR_TYPE_INT64:
		anotc_add_long(&frame, *((long long*)value));
		break;
	case ANOTC_PAR_TYPE_FLOAT:
		anotc_add_float(&frame, *((float*)value));
		break;
	case ANOTC_PAR_TYPE_DOUBLE:
		anotc_add_double(&frame, *((double*)value));
		break;
	case ANOTC_PAR_TYPE_STRING:
		anotc_add_string(&frame, ((char*)value));
		break;
	default:
		break;
	}
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}