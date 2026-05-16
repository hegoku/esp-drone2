#ifndef PLATFORM_DRIVERS_WIFI_H
#define PLATFORM_DRIVERS_WIFI_H

#define WIFI_PACK_MAX_LEN 200

struct wifi_pack{
	unsigned char data[WIFI_PACK_MAX_LEN];
	int size;
	char priority;
};

void wifi_set_name(char *name);
char* wifi_get_name();
void wifi_set_password(char *pwd);
char* wifi_get_password();
void wifi_set_udp_port(unsigned short port);
unsigned short *wifi_get_udp_port();

int init_wifi();
void wifi_set_recv_handler(void (*handler)(unsigned char *data, int len));
void wifi_send(unsigned char *data, int len);
int wifi_flush();
short int wifi_get_rssi();
#endif