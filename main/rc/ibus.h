#ifndef RC_IBUS_H
#define RC_IBUS_H

#include "rc/rc.h"

void init_ibus();
void ibus_hw_init();
void ibus_decode(unsigned char *data, int count);

extern struct rc_protocol ibus;
#endif