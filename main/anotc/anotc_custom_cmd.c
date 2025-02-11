#include "anotc/anotc_official_frame.h"
#include "anotc/anotc_cmd_frame.h"

struct anotc_cmd_info {
	unsigned int cid;
	char* (*handle)(unsigned char *value);
};

#include "anotc_cmd_list.c"

struct anotc_cmd_info* anotc_get_custom_cmd(unsigned int cid)
{
	for (unsigned int i=0;i<ANOTC_CMD_LIST_SIZE;i++) {
		if (cmd_list[i].cid==cid) {
			return &cmd_list[i];
		}
	}
	return NULL;
}


void anotc_cmd_frame_send_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac)
{
	unsigned int cid = (((unsigned int)frame->frame.data[0])<<16) | (((unsigned short)frame->frame.data[1])<<8) | frame->frame.data[2];
	struct anotc_cmd_info* info;
	char *res = 0;
	info = anotc_get_custom_cmd(cid);
	if (info==NULL) return;
	res = info->handle(&(frame->frame.data[3]));
	if (res==0) {
		anotc_send_frame_check(ANOTC_FRAME_CMD_SEND, sc, ac, 0, 0);
	} else {
		anotc_send_frame_check(ANOTC_FRAME_CMD_SEND, sc, ac, 1, res);
	}
}