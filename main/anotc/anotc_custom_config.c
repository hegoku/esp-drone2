#include <string.h>
#include "flight/flight.h"
#include "anotc/anotc.h"
#include "anotc/anotc_official_frame.h"
#include "anotc/anotc_config_frame.h"

struct anotc_config_info {
	unsigned short par_id;
	enum ANOTC_PAR_TYPE type;
	char par_name[20];
	char *par_info;
	char* (*set)(void *value);
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
		if (val==0) {
			anotc_send_device_info(ANOTC_DEVICE_ADDR, 1, 2, 0, 8, "esp32-drone");
			flight.system_info.drone_center_connect = 1;
		} else if (val==1) {
			unsigned char sensor_status = 0;
			if (flight.imu.status==IMU_STATUS_ON) {
				sensor_status = 1;
			}
			if (IS_COMPASS_ON(flight.compass)) {
				sensor_status |= 0x2;
			}
			if (IS_BARO_ON(flight.baro)) {
				sensor_status |= 0x4;
			}
			anotc_send_device_info(sensor_status, 1, 2, 0, 8, "esp32-drone");
			flight.system_info.drone_center_connect = 1;
		} else if (val==2) {
			flight.system_info.drone_center_connect = 0;
		}
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
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_CMD, sc, ac, 0, 0);
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
	char *res = 0;
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
		res = info->set((void*)(&(frame->frame.data[2])));
		if (res==0) {
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac, 0, 0);
		} else {
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac, 1, res);
		}
		break;
	case ANOTC_PAR_TYPE_STRING:
		config_begin_transaction();
		char *buf = malloc(frame->frame.len-sizeof(unsigned short)+1);
		memcpy((void*)buf, (void*)(&(frame->frame.data[2])), frame->frame.len-sizeof(unsigned short));
		buf[frame->frame.len-sizeof(unsigned short)] = '\0';
		res = info->set((void*)buf);
		free(buf);
		if (res==0) {
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac, 0, 0);
		} else {
			anotc_send_frame_check(ANOTC_FRAME_CONFIG_READ_WRITE, sc, ac, 1, res);
		}
		break;
	default:
		break;
	}
	
}