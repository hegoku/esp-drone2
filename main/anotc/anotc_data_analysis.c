#include "data_analysis/data_analysis.h"
#include "anotc/anotc.h"
#include "anotc/anotc_config_frame.h"
#include "anotc/anotc_official_frame.h"
#include "flight/flight.h"

unsigned int da_send_interval_ms = 65536;
static unsigned short _send_list[25] = {0};
static unsigned short _send_list_size = 0;

void anotc_send_data_analytics_count(unsigned short ms, unsigned short count)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_DATA_ANALYSIS_LIST_CMD;

	frame.data[frame.len++] = ANOTC_CONFIG_DATA_ANALYSIS_LIST_COUNT;
	anotc_add_ushort(&frame, count);
	anotc_add_ushort(&frame, ms);
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_data_analytics_info(unsigned short index, enum DATA_ANALYSIS_ITEM_TYPE type, const char *name, const char *info)
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_DATA_ANALYSIS_LIST_CMD;

	frame.data[frame.len++] = ANOTC_CONFIG_DATA_ANALYSIS_LIST_ITEM_INFO;
	anotc_add_ushort(&frame, index);
	frame.data[frame.len++] = type;
	strncpy((char *)&(frame.data[frame.len]), name, 20);
	frame.len += 20;
	anotc_add_string(&frame, info);
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void anotc_send_data_analytics()
{
	struct anotc_frame frame;
	const struct data_analysis_item *item;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_DATA_ANALYSIS_LIST_CMD;

	anotc_add_uint(&frame, flight.system_info.time);
	for (int i=0; i<_send_list_size; i++) {
		item = da_item_at(_send_list[i]);
		if (item==NULL) continue;
		switch (item->type)
		{
		case DATA_TYPE_UINT8:
			frame.data[frame.len++] = *((unsigned char*)item->variable);
			break;
		case DATA_TYPE_INT8:
			frame.data[frame.len++] = *((char*)item->variable);
			break;
		case DATA_TYPE_UINT16:
			anotc_add_ushort(&frame, *((unsigned short*)item->variable));
			break;
		case DATA_TYPE_INT16:
			anotc_add_short(&frame, *((short*)item->variable));
			break;
		case DATA_TYPE_UINT32:
			anotc_add_uint(&frame, *((unsigned int*)item->variable));
			break;
		case DATA_TYPE_INT32:
			anotc_add_int(&frame, *((int*)item->variable));
			break;
		case DATA_TYPE_UINT64:
			anotc_add_ulong(&frame, *((unsigned long long*)item->variable));
			break;
		case DATA_TYPE_INT64:
			anotc_add_long(&frame, *((long long*)item->variable));
			break;
		case DATA_TYPE_FLOAT:
			anotc_add_float(&frame, *((float*)item->variable));
			break;
		case DATA_TYPE_DOUBLE:
			anotc_add_double(&frame, *((double*)item->variable));
			break;
		case DATA_TYPE_STRING:
			anotc_add_string(&frame, ((char*)item->variable));
			break;
		default:
			break;
		}
	}
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}

void _anotc_handle_data_analytics_set(unsigned short ms, unsigned char *buff, int len)
{
	unsigned short index = 0;
	unsigned short size = 0;
	for (int i=0; i< len; i++) {
		if (i%2==0) {
			index = buff[i];
		} else {
			index |= (unsigned short)buff[i]<<8;
			_send_list[size++] = index;
		}
	}
	da_send_interval_ms = ms;
	_send_list_size = size;
}

void anotc_data_analytics_frame_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac)
{
	unsigned char cmd = frame->frame.data[0];
	unsigned char *data = &(frame->frame.data[1]);
	switch (cmd)
	{
	case ANOTC_CONFIG_DATA_ANALYSIS_LIST_COUNT:
		anotc_send_data_analytics_count((unsigned short)da_send_interval_ms, data_analysis_item_count());
		break;
	case ANOTC_CONFIG_DATA_ANALYSIS_LIST_ITEM_INFO:
		unsigned short val = (data[1]<<8) | data[0];
		const struct data_analysis_item *item;
		item = da_item_at(val);
		if (item==NULL) return;
		anotc_send_data_analytics_info(val, item->type, item->name, item->info);
		break;
	case ANOTC_CONFIG_DATA_ANALYSIS_LIST_SET:
		unsigned short ms = (data[1]<<8) | data[0];
		data+=2;
		_anotc_handle_data_analytics_set(ms, data, frame->frame.len-3);
		anotc_send_frame_check(ANOTC_FRAME_DATA_ANALYSIS_LIST_CMD, sc, ac, 0, 0);
		break;
	default:
		break;
	}
}
