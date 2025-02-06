#ifndef CLOCKSOURCE_CLOCKSOURCE_H
#define CLOCKSOURCE_CLOCKSOURCE_H

struct clocksource {
	char *name;
	unsigned int freq;
	void (*start)();
};

void sys_timer_set(struct clocksource *source);
void sys_timer_start();
struct clocksource* sys_timer_get();

#endif