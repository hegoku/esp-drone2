#ifndef BUS_BUS_H
#define BUS_BUS_H

#define BUS_MAX_PROBS_NR 10

struct bus_dev;

struct bus {
	char *name;
	void *priv;
	void (*init)(struct bus *bus);
	int (*(*probs)[BUS_MAX_PROBS_NR])(struct bus_dev *dev);
	struct bus_dev *dev_list;
};

struct bus_dev {
	struct bus *bus;
	char *name;
	int address;
	void *priv;
	struct bus_dev *next;
};

void bus_init(struct bus *bus_list, int len);
void bus_add_dev(struct bus *bus, struct bus_dev *dev);
#endif