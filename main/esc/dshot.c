#include "esc/dshot.h"

unsigned char dshot_checksum(unsigned short data)
{
	unsigned short csum = 0;

	for (int i = 0; i < 3; i++)
	{
		csum ^= data;
		data >>= 4;
	}

	return csum & 0xf;
}

unsigned short dshot_packet(unsigned short value, unsigned char telemetry)
{
	value <<= 1;
	value |= telemetry;

	value = (value << 4) | dshot_checksum(value);

	return value;
}