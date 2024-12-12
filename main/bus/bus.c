#include "bus/bus.h"

void init_bus_driver(struct bus *bus_list, int len)
{
	for (int i=0;i<len;i++) {
		bus_list[i].init(&bus_list[i]);
	}
}