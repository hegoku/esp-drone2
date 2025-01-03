#ifndef ANOTC_H
#define ANOTC_H

#define ANOTC_DATA_MAX_SIZE 512
#define ANOTC_V8_HEAD_SIZE 7
#define ANOTC_V8_HEAD 0xAB

struct anotc_frame {
	unsigned char head;
	unsigned char s_addr;
	unsigned char d_addr;
	unsigned char fun;
	unsigned short len;
	unsigned char data[ANOTC_DATA_MAX_SIZE];
}__attribute__ ((__packed__));

union _un_anotc_v8_frame{
    struct anotc_frame frame;
    unsigned char rawBytes[sizeof(struct anotc_frame)];
};

#define PREPARE_ANOTC_FRAME(frame) { \
frame.head = ANOTC_V8_HEAD;\
frame.s_addr = 0xAF;\
frame.d_addr = 0x01;\
frame.len = 0;\
}

static inline void anotc_add_float(struct anotc_frame *frame, float data)
{
	*(float*)(&(frame->data[frame->len])) = data;
	frame->len += sizeof(float);
}

static inline void anotc_add_short(struct anotc_frame *frame, short data)
{
	*(short int*)(&(frame->data[frame->len])) = data;
	frame->len += sizeof(short int);
}

static inline void anotc_add_ushort(struct anotc_frame *frame, unsigned short data)
{
	*(unsigned short int*)(&(frame->data[frame->len])) = data;
	frame->len += sizeof(unsigned short int);
}

static inline void anotc_add_int(struct anotc_frame *frame, int data)
{
	*(int*)(&(frame->data[frame->len])) = data;
	frame->len += sizeof(int);
}

static inline void anotc_add_uint(struct anotc_frame *frame, unsigned int data)
{
	*(unsigned int*)(&(frame->data[frame->len])) = data;
	frame->len += sizeof(unsigned int);
}

static inline void anotc_add_checksum(struct anotc_frame *frame)
{
	frame->data[frame->len] = 0;
	frame->data[frame->len + 1] = 0;
	for (int i = 0; i < ANOTC_V8_HEAD_SIZE + frame->len; i++)
	{
		frame->data[frame->len]+=((unsigned char *)frame)[i];
		frame->data[frame->len+1] += frame->data[frame->len];
	}
}

struct anotc_cb {
	void (*cmd_handle)(unsigned char cid, unsigned char *cmd, unsigned char sum_check, unsigned char add_check);
	void (*read_config_handle)(unsigned short int par_id);
	void (*write_config_handle)(unsigned short int par_id, int val, unsigned char sum_check, unsigned char add_check);
	void (*custom_cmd_handle)(unsigned char cid, unsigned char *cmd, unsigned char sum_check, unsigned char add_check);
	void (*func_0x40_handle)(unsigned char *data);
};

extern void (*_anotc_send_func)(unsigned char *data, int len);

void register_anotc_cb(struct anotc_cb *cb);
void anotc_set_send_func(void (*func)(unsigned char *data, int len));
void anotc_decode(unsigned char *data, int count);

#endif