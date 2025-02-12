#ifndef MIXER_DSHOT_H
#define MIXER_DSHOT_H

#include "esc/dshot.h"
#include "mixer/mixer.h"

#define MIXER_DSHOT_THROTTLE_MIN 48
#define MIXER_DSHOT_THROTTLE_MAX 2047

void mixer_dshot_write(struct mixer *mixer);
void mixer_dshot_init(struct mixer *mixer);

unsigned short dshot_convert_throttle(unsigned short throttle);

extern struct esc_protocol mixer_dshot600;
#endif