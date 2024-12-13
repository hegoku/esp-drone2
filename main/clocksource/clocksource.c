#include "clocksource/clocksource.h"

static void (*_start_sys_timer)();

void sys_timer_set(void (*start_func)())
{
	_start_sys_timer = start_func;
}

void sys_timer_start()
{
	_start_sys_timer();
}