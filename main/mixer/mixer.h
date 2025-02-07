#ifndef MIXER_MIXER_H
#define MIXER_MIXER_H

struct mixer_moter {
	unsigned int wire;
	unsigned short value;
	void *priv;
};

struct esc_protocol {
	char *name;
	void *priv;
	void (*init)(struct mixer_moter *motor, int count);
	void (*write)(struct mixer_moter *motor, int count);
};

struct mixer {
	struct esc_protocol *esc_protocol;
	struct mixer_moter motor[4];
};

void init_mixer();
void mixer_output(struct mixer *mixer);

#endif