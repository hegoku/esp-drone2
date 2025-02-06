#include "clocksource/clocksource.h"

static struct clocksource *_sys_timer;

void sys_timer_set(struct clocksource *source)
{
	_sys_timer = source;
}

void sys_timer_start()
{
	_sys_timer->start();
}

struct clocksource* sys_timer_get()
{
	return _sys_timer;
}