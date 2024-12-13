#ifndef BUS_BUS_H
#define BUS_BUS_H

#define BUS_MAX_PROBS_NR 10

struct bus_dev;

struct bus {
	char *name;
	void *priv;
	void (*init)(struct bus *bus);
	int (*(*probs)[BUS_MAX_PROBS_NR])(struct bus_dev *dev);
};

struct bus_dev {
	struct bus *bus;
	int address;
	void *priv;
};

void bus_init(struct bus *bus_list, int len);

#endif