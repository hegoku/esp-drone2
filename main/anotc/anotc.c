#include "anotc/anotc.h"

void (*_anotc_send_func)(unsigned char *data, int len);

void anotc_set_send_func(void (*func)(unsigned char *data, int len))
{
	_anotc_send_func = func;
}