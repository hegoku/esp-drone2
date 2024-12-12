#ifndef BUS_BUS_H
#define BUS_BUS_H

struct bus {
	char *name;
	void *priv;
	int (*init)(struct bus *bus);
	int (*(*probs)[10])(struct bus *bus);
	int (*read)(struct bus_dev *dev, unsigned char *buf, int bytes);
	int (*write)(struct bus_dev *dev, unsigned char *buf, int bytes);
};

struct bus_dev {
	struct bus *bus;
	int address;
	void *priv;
};

void bus_init(struct bus *bus_list, int len);

#endif