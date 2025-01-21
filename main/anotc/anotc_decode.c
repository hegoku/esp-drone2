#include "anotc/anotc.h"
#include "anotc/anotc_config_frame.h"

enum anotc_decode_status
{
	HEAD,
	SRC_ADDR,
	DST_ADDR,
	FUNC,
	LEN1,
	LEN2,
	READ_DATA,
	SUM_CHECK,
	ADD_CHECK
};

struct anotc_decode_data
{
	enum anotc_decode_status status;
	unsigned short frame_index;
	unsigned short frame_read_count;
	unsigned char sum_check;
	unsigned char add_check;
	union _un_anotc_v8_frame frame;
};

static struct anotc_decode_data _decode_data = {0};

struct anotc_cb *anotc_callback = 0;

static inline int _sum_check(struct anotc_frame *frame, unsigned char sum_check, unsigned char add_check);

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
			if (data[i]!=ANOTC_V8_HEAD) continue; //找帧头
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = ANOTC_V8_HEAD;
			_decode_data.status = SRC_ADDR;
		} else if (_decode_data.status==SRC_ADDR) { //源地址
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.status = DST_ADDR;
		} else if (_decode_data.status==DST_ADDR) { //目标地址
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.status = FUNC;
		} else if (_decode_data.status == FUNC) { //func
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.status = LEN1;
		} else if (_decode_data.status == LEN1) {
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.status = LEN2;
		} else if (_decode_data.status == LEN2) {
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.status = READ_DATA;
		} else if (_decode_data.status == READ_DATA) { //read data
			_decode_data.frame.rawBytes[_decode_data.frame_index++] = data[i];
			_decode_data.frame_read_count++;
			if (_decode_data.frame_read_count==_decode_data.frame.frame.len) {
				_decode_data.status = SUM_CHECK;
			}
		} else if (_decode_data.status==SUM_CHECK) { //sc
			_decode_data.sum_check = data[i];
			_decode_data.status = ADD_CHECK;
		} else if (_decode_data.status==ADD_CHECK) { //ac
			_decode_data.add_check = data[i];

			//handle data
			real_data = &_decode_data.frame.rawBytes[ANOTC_V8_HEAD_SIZE];
			if (_sum_check(&_decode_data.frame.frame, _decode_data.sum_check, _decode_data.add_check))
			{
				switch (_decode_data.frame.frame.fun)
				{
				case ANOTC_FRAME_CONFIG_CMD:
				{
					anotc_config_frame_cmd_handler(real_data);
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

static inline int _sum_check(struct anotc_frame *frame, unsigned char sum_check, unsigned char add_check)
{
    unsigned char s = 0;
    unsigned char a = 0;
    for (int i=0; i<ANOTC_V8_HEAD_SIZE+frame->len; i++) {
        s+=((unsigned char *)frame)[i];
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