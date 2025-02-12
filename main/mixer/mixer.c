#include "misc/config.h"
#include "flight/flight.h"
#include "mixer/mixer.h"
#include "mixer/dshot.h"

extern struct mixer __mixer;

void init_mixer()
{
	unsigned char protocol = 0;
	config_read_uchar("esc.protocol", &protocol);
	flight.mixer = &__mixer;
	switch (protocol)
	{
	case 0:
		__mixer.esc_protocol = &mixer_dshot600;
		break;
	default:
		break;
	}
	
	flight.mixer->esc_protocol->init(flight.mixer);
}

void mixer_output(struct mixer *mixer)
{
	mixer->esc_protocol->write(mixer);
}