#ifndef RC_PC_H
#define RC_PC_H

#include "rc/rc.h"

void pc_rc_receive(unsigned short *channel);

extern struct rc_protocol pc;
#endif