#ifndef ESC_DSHOT_H
#define ESC_DSHOT_H

struct dshot_protocol;

#define DSHOT_GET_STRUCT(x) ((struct dshot_protocol*)x)

void init_dshot(struct dshot_protocol *dshot);
unsigned char dshot_checksum(unsigned short data);
unsigned short dshot_packet(unsigned short value, unsigned char telemetry);
void dshot_write(struct dshot_protocol *dshot, unsigned short value, unsigned char telemetry);

#endif