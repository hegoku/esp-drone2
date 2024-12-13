#ifndef CLOCKSOURCE_CLOCKSOURCE_H
#define CLOCKSOURCE_CLOCKSOURCE_H

void sys_timer_set(void (*start_func)());
void sys_timer_start();

#endif