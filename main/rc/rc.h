#ifndef RC_RC_H
#define RC_RC_H

struct rc;

struct rc_protocol {
	char *name;
	void *priv;
	void (*init)(struct rc *rc);
	void (*hw_init)(struct rc *rc);
};

struct rc {
	struct rc_protocol *protocol;
	unsigned short channel[14];
};

#endif