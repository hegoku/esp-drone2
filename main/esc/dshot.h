#ifndef ESC_DSHOT_H
#define ESC_DSHOT_H

unsigned char dshot_checksum(unsigned short data);
unsigned short dshot_packet(unsigned short value, unsigned char telemetry);

#endif