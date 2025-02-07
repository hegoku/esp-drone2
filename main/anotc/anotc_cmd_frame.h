#ifndef ANOTC_CMD_FRAME_H
#define ANOTC_CMD_FRAME_H

#include "anotc/anotc.h"

#define ANOTC_FRAME_CMD_SEND 0xC0

void anotc_cmd_frame_send_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac);

#endif