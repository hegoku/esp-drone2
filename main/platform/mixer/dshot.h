#ifndef PLATFORM_MIXER_DSHOT_H
#define PLATFORM_MIXER_DSHOT_H

#include <driver/rmt_tx.h>

struct dshot_protocol_priv {
	rmt_sync_manager_handle_t synchro;
	rmt_channel_handle_t *rmt_channel;
	unsigned int channel_count;
	
};

#define MIXER_DSHOT_GET_PRIV(x) ((struct dshot_protocol_priv*)(x))

#endif