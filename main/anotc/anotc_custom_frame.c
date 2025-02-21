#include "anotc/anotc.h"
#include "anotc/anotc_custom_frame.h"
#include "flight/flight.h"

void anotc_send_system_info()
{
	struct anotc_frame frame;
	PREPARE_ANOTC_FRAME(frame);
	frame.fun = ANOTC_FRAME_CUSTOM_SYSTEM_INFO;

	frame.data[frame.len++] = flight.status;
	frame.data[frame.len++] = flight.system_info.cpu_load;
	anotc_add_float(&frame, flight.battery.voltage);
	anotc_add_float(&frame, flight.battery.current);
	frame.data[frame.len++] = flight.rc.status;
	anotc_add_checksum(&frame);
	_anotc_send_func((unsigned char *)&frame, ANOTC_V8_HEAD_SIZE + frame.len + 2);
}