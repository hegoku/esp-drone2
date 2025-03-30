#include <math.h>
#include <string.h>
#include <driver/ledc.h>
#include "mixer/mixer.h"
#include "platform/mixer/bushed.h"

void mixer_bushed_init(struct mixer *mixer)
{
	ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 15000,					// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_3,			// timer index
    };
	ledc_timer_config(&ledc_timer);

	int count = sizeof(mixer->motor) / (sizeof(mixer->motor[0]));

	struct bushed_protocol_motor_priv *m_priv;
	for (int i = 0; i < count; i++)
	{
		m_priv = (struct bushed_protocol_motor_priv*)malloc(sizeof(struct bushed_protocol_motor_priv));
		memset(m_priv, 0, sizeof(struct bushed_protocol_motor_priv));
		mixer->motor[i]->priv = m_priv;
		m_priv->channel.gpio_num = mixer->motor[i]->wire;
		m_priv->channel.duty = 0;
		m_priv->channel.speed_mode = LEDC_LOW_SPEED_MODE;
		m_priv->channel.timer_sel = LEDC_TIMER_3;
		m_priv->channel.channel = 4+i;
		ledc_channel_config(&(m_priv->channel));
	}
}

void mixer_bushed_write(struct mixer *mixer)
{
	struct bushed_protocol_motor_priv *priv;
	int count = sizeof(mixer->motor) / (sizeof(mixer->motor[0]));
	for (int i=0;i<count;i++) {
		priv = BUSHED_GET_MOTOR_PRIV(mixer->motor[i]->priv);
		ledc_set_duty(priv->channel.speed_mode, priv->channel.channel, (uint32_t)round(mixer->motor[i]->value/1000.0*255.0));
	}
	for (int i=0;i<count;i++) {
		priv = BUSHED_GET_MOTOR_PRIV(mixer->motor[i]->priv);
		ledc_update_duty(priv->channel.speed_mode, priv->channel.channel);
	}
}

struct esc_protocol mixer_bushed = {
	.name = "Bushed",
	.init = mixer_bushed_init,
	.write = mixer_bushed_write
};