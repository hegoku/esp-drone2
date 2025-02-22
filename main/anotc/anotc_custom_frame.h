#ifndef ANOTC_CUSTOM_FRAME_H
#define ANOTC_CUSTOM_FRAME_H

#define ANOTC_FRAME_CUSTOM_SYSTEM_INFO 0xF1
#define ANOTC_FRAME_CUSTOM_PID 0xF2

void anotc_send_system_info();
void anotc_send_pid();
#endif