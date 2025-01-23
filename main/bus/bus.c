#include "bus/bus.h"

extern int (*spi_dev_probs[])(struct bus_dev *dev);

void bus_init(struct bus *bus_list, int len)
{
	for (int i=0;i<len;i++) {
		bus_list[i].init(&bus_list[i]);
	}
}

void bus_add_dev(struct bus *bus, struct bus_dev *dev)
{
	dev->next = bus->dev_list;
	bus->dev_list = dev;
}