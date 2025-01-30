#include <string.h>
#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"
#include "anotc/anotc_config_frame.h"

struct anotc_config_info {
	unsigned short par_id;
	enum ANOTC_PAR_TYPE type;
	char par_name[20];
	char *par_info;
	void (*set)(void *value);
	void* (*get)();
};

#include "anotc/anotc_configuration_list.c"

struct anotc_config_info* anotc_get_custom_config(unsigned short par_id)
{
	for (unsigned int i=0;i<ANOTC_CONFIGURATION_LIST_SIZE;i++) {
		if (configuration_list[i].par_id==par_id) {
			return &configuration_list[i];
		}
	}
	return NULL;
}

void anotc_config_frame_read_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac)
{
	unsigned char cmd = frame->frame.data[0];
	unsigned short val = (frame->frame.data[2]<<8) | frame->frame.data[1];
	struct anotc_config_info* info;
	switch (cmd)
	{
	case ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO:
		anotc_send_device_info(ANOTC_DEVICE_ADDR, 1, 2, 0, 8, "esp32-drone");
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_COUNT:
		anotc_send_config_count(ANOTC_CONFIGURATION_LIST_SIZE);
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_VALUE:
		info = anotc_get_custom_config(val);
		if (info==NULL) return;
		anotc_send_config_value(val, info->type, info->get());
		break;
	case ANOTC_CONFIG_FRAME_CMD_READ_INFO:
		info = anotc_get_custom_config(val);
		if (info==NULL) return;
		anotc_send_config_info(info->par_id, info->type, info->par_name, info->par_info);
		break;
	case ANOTC_CONFIG_FRAME_CMD_0X10:
		switch (val)
		{
		case ANOTC_CONFIG_FRAME_VAL_SAVE_PARAM:
			config_commit();
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_CMD, sc, ac);
			break;
		
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void anotc_config_frame_write_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac)
{
	unsigned short par_id = (frame->frame.data[1]<<8) | frame->frame.data[0];
	struct anotc_config_info* info;
	info = anotc_get_custom_config(par_id);
	if (info==NULL) return;
	switch (info->type)
	{
	case ANOTC_PAR_TYPE_UINT8:
	case ANOTC_PAR_TYPE_INT8:
	case ANOTC_PAR_TYPE_UINT16:
	case ANOTC_PAR_TYPE_INT16:
	case ANOTC_PAR_TYPE_UINT32:
	case ANOTC_PAR_TYPE_INT32:
	case ANOTC_PAR_TYPE_UINT64:
	case ANOTC_PAR_TYPE_INT64:
	case ANOTC_PAR_TYPE_FLOAT:
	case ANOTC_PAR_TYPE_DOUBLE:
		config_begin_transaction();
		info->set((void*)(&(frame->frame.data[2])));
		anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac);
		break;
	case ANOTC_PAR_TYPE_STRING:
		config_begin_transaction();
		char *buf = malloc(frame->frame.len-sizeof(unsigned short)+1);
		memcpy((void*)buf, (void*)(&(frame->frame.data[2])), frame->frame.len-sizeof(unsigned short));
		buf[frame->frame.len-sizeof(unsigned short)] = '\0';
		info->set((void*)buf);
		free(buf);
		anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac);
		break;
	default:
		break;
	}
	
}