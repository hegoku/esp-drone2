#ifndef ANOTC_DATA_ANALYTICS_H
#define ANOTC_DATA_ANALYTICS_H

extern unsigned int da_send_interval_ms;

void anotc_data_analytics_frame_cmd_handler(union _un_anotc_v8_frame *frame, unsigned char sc, unsigned char ac);

void anotc_send_data_analytics();
#endif