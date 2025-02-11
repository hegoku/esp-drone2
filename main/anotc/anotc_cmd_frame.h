#ifndef ANOTC_CMD_FRAME_H
#define ANOTC_CMD_FRAME_H

#include "anotc/anotc.h"

#define ANOTC_FRAME_CMD_SEND 0xC0

#define ANOTC_CMD_CALIBRATE_GYRO 0x0
#define ANOTC_CMD_CALIBRATE_ACCEL 0x1

void anotc_send_cmd_response(unsigned int cid, unsigned char code, unsigned char *data, int len);
void anotc_cmd_frame_send_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac);

#endif