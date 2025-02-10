#include <stdlib.h>
#include "mixer/dshot.h"
#include "platform/esc/dshot.h"

void mixer_dshot_write(struct mixer_moter *motor, int count)
{
	struct dshot_protocol *priv;
	for (int i=0;i<count;i++) {
		priv = DSHOT_GET_STRUCT(motor[i].priv);
		dshot_write(priv, motor[i].value, 0);
	}
}
	
 
void mixer_dshot_init(struct mixer_moter *motor, int count)
{
	struct dshot_protocol *priv;
	for (int i=0;i<count;i++) {
		priv = (struct dshot_protocol*)malloc(sizeof(struct dshot_protocol));
		motor[i].priv = priv;
		priv->gpio_num = motor[i].wire;
		init_dshot(priv);
	}
	
}

struct esc_protocol mixer_dshot600 = {
	.name = "DSHOT600",
	.init = mixer_dshot_init,
	.write = mixer_dshot_write
};