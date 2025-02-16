#include <stdlib.h>
#include "mixer/dshot.h"
#include "platform/mixer/dshot.h"
#include "platform/esc/dshot.h"

void mixer_dshot_write(struct mixer *mixer)
{
	struct dshot_protocol_motor *priv;
	int count = sizeof(mixer->motor) / (sizeof(mixer->motor[0]));
	for (int i=0;i<count;i++) {
		priv = DSHOT_GET_MOTOR_PRIV(mixer->motor[i]->priv);
		dshot_write(priv, dshot_convert_throttle(mixer->motor[i]->value), 0);
	}
}

void mixer_dshot_init(struct mixer *mixer)
{
	struct dshot_protocol_priv *priv = (struct dshot_protocol_priv*)malloc(sizeof(struct dshot_protocol_priv));
	struct dshot_protocol_motor *m_priv;
	int count = sizeof(mixer->motor) / (sizeof(mixer->motor[0]));

	priv->channel_count = count;
	priv->rmt_channel = (rmt_channel_handle_t *)malloc(sizeof(rmt_channel_handle_t)*count);

	mixer->esc_protocol->priv = (void*)priv;

	for (int i=0;i<count;i++) {
		m_priv = (struct dshot_protocol_motor*)malloc(sizeof(struct dshot_protocol_motor));
		m_priv->rmt_channel = &(priv->rmt_channel[i]);
		mixer->motor[i]->priv = m_priv;
		m_priv->gpio_num = mixer->motor[i]->wire;
		dshot_init(m_priv);
	}
	
	rmt_sync_manager_config_t synchro_config = {
		.tx_channel_array = priv->rmt_channel,
		.array_size = count,
	};
	rmt_new_sync_manager(&synchro_config, &(priv->synchro));
	
	for (int j=0;j<50;j++) {
		for (int i=0;i<count;i++) {
			m_priv = DSHOT_GET_MOTOR_PRIV(mixer->motor[i]->priv);
			dshot_write(m_priv, 0, 0);
		}
	}
}

struct esc_protocol mixer_dshot600 = {
	.name = "DSHOT600",
	.init = mixer_dshot_init,
	.write = mixer_dshot_write
};