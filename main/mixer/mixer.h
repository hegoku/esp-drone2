#ifndef MIXER_MIXER_H
#define MIXER_MIXER_H

#define MIXER_THROTTLE_MAX 1000
#define MIXER_THROTTLE_MIN 0

#define MOTOR_MAPPING_VALUE(x,i) ((x & (0xFUL<<(i*4)))>>(i*4))
#define MOTOR_GET_INDEX(v) (v & 0x7)
#define MOTOR_IS_REVERSE(v) (v & 0x8)

#define MOTOR_SET_REVERSE(i,r) (r==0?(i):(i|0x8U))
#define MOTOR_PUT_MAPPING_VALUE(v, p,i,r) (v | (MOTOR_SET_REVERSE(i,r)<<(p*4)))

struct mixer;

struct mixer_moter {
	unsigned int wire;
	unsigned short value;
	void *priv;
};

struct esc_protocol {
	char *name;
	void *priv;
	void (*init)(struct mixer *mixer);
	void (*write)(struct mixer *mixer);
};

struct mixer {
	struct esc_protocol *esc_protocol;
	struct mixer_moter *motor[4];
};

void init_mixer();
void mixer_output(struct mixer *mixer);

extern struct mixer_moter __motors[4];

#endif