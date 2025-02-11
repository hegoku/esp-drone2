#include "anotc/anotc.h"
#include "platform/drivers/anotc_wifi.h"
#include "platform/drivers/wifi.h"

void anotc_wifi_send(unsigned char *data, int len)
{
	wifi_send(data, len);
}

int anotc_wifi_init()
{
	wifi_set_recv_handler(anotc_decode);
	anotc_set_send_func(anotc_wifi_send);
	return 0;
}