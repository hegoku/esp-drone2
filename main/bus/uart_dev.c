#include "bus/bus.h"


int (*uart_dev_probs[BUS_MAX_PROBS_NR])(struct bus_dev *dev) = {

};