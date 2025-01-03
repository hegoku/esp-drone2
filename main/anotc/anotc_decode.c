#include "anotc/anotc.h"

enum anotc_decode_status
{
	HEAD,
	DST_ADDR,
	FUNC,
	LEN,
	READ_DATA,
	SUM_CHECK,
	ADD_CHECK
};

struct anotc_decode_data
{
	enum anotc_decode_status status;
	unsigned char frame_index;
	unsigned char frame_read_count;
};

static struct anotc_decode_data _decode_data = {0};

struct anotc_cb *anotc_callback = 0;
static unsigned char frame[23] = {0};
unsigned char func = 0;
unsigned char sum_check = 0;
unsigned char add_check = 0;

static inline int _sum_check(unsigned char *data, unsigned char len, unsigned char sum_check, unsigned char add_check);

void register_anotc_cb(struct anotc_cb *cb)
{
	anotc_callback = cb;
}

void anotc_decode(unsigned char *data, int count)
{
	unsigned char *real_data = 0;
	for (int i = 0; i < count; i++)
	{
		if (_decode_data.status==HEAD) {
			if (data[i]!=0xAA) continue; //找帧头
			frame[_decode_data.frame_index++] = 0xAA;
			_decode_data.status = DST_ADDR;
		} else if (_decode_data.status==DST_ADDR) { //目标地址
			frame[_decode_data.frame_index++] = data[i];
			_decode_data.status = FUNC;
		} else if (_decode_data.status == FUNC) { //func
			frame[_decode_data.frame_index++] = data[i];
			func = data[i];
			_decode_data.status = LEN;
		} else if (_decode_data.status == LEN) { //len
			frame[_decode_data.frame_index++] = data[i];
			_decode_data.status = READ_DATA;
		} else if (_decode_data.status == READ_DATA) { //read data
			frame[_decode_data.frame_index++] = data[i];
			_decode_data.frame_read_count++;
			if (_decode_data.frame_read_count==frame[3]) {
				_decode_data.status = SUM_CHECK;
			}
		} else if (_decode_data.status==SUM_CHECK) { //sc
			sum_check = data[i];
			_decode_data.status = ADD_CHECK;
		} else if (_decode_data.status==ADD_CHECK) { //ac
			add_check = data[i];

			//handle data
			real_data = &frame[4];
			if (_sum_check(frame, frame[3] + 4, sum_check, add_check))
			{
				switch (func)
				{
				case 0x40: //遥控器
				{
					if (anotc_callback!=0 && anotc_callback->func_0x40_handle!=0) {
						anotc_callback->func_0x40_handle(real_data);
					}
					break;
				}
				case 0xE0:
				{
					if (anotc_callback!=0 && anotc_callback->cmd_handle!=0) {
						anotc_callback->cmd_handle(real_data[0], &real_data[1], sum_check, add_check);
					}
					break;
				}
                case 0xE1:
                {
					unsigned short int par_id;
					par_id = (real_data[1]<<8) | real_data[0];
					if (anotc_callback!=0 && anotc_callback->read_config_handle!=0) {
						anotc_callback->read_config_handle(par_id);
					}
					break;
                }
                case 0xE2:
                {
                    unsigned short int par_id;
					int val;
					par_id = (real_data[1]<<8) | real_data[0];
					val = (real_data[5]<<24) | (real_data[4]<<16) | (real_data[3]<<8) | real_data[2];
					if (anotc_callback != 0 && anotc_callback->write_config_handle != 0)
					{
						anotc_callback->write_config_handle(par_id, val, sum_check, add_check);
					}
					break;
                }
				case 0xE3: //custom cmd
				{
					if (anotc_callback!=0 && anotc_callback->cmd_handle!=0) {
						anotc_callback->custom_cmd_handle(real_data[0], &real_data[1], sum_check, add_check);
					}
					break;
				}
                default:
                    break;
				}
			}

			_decode_data.status = HEAD;
			_decode_data.frame_index = 0;
			_decode_data.frame_read_count = 0;
		}
	}
}

static inline int _sum_check(unsigned char *data, unsigned char len, unsigned char sum_check, unsigned char add_check)
{
    unsigned char s = 0;
    unsigned char a = 0;
    for (unsigned char i=0; i<len; i++) {
        s+=data[i];
        a+=s;
    }
	if (s == sum_check && a == add_check)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}