#include "bus/bus.h"

void bus_init(struct bus *bus_list, int len)
{
	for (int i=0;i<len;i++) {
		bus_list[i].init(&bus_list[i]);
	}
}