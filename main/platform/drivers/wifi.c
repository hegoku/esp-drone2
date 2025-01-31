#include <string.h>
#include <esp_wifi.h>
#include "platform/drivers/wifi.h"
#include "misc/config.h"

static char wifi_name[32];
static char wifi_password[64];

wifi_config_t wifi_config;

void wifi_set_name(char *name)
{
	strcpy(wifi_name, name);
	config_write_string("wifi_name", wifi_name);
}

char* wifi_get_name()
{
	return wifi_name;
}

void wifi_set_password(char *pwd)
{
	strcpy(wifi_password, pwd);
	config_write_string("wifi_pwd", wifi_password);
}

char* wifi_get_password()
{
	return wifi_password;
}

void init_wifi()
{
	config_read_string("wifi_name", wifi_name);
	config_read_string("wifi_pwd", wifi_password);
	strcpy((char*)wifi_config.sta.ssid, wifi_name);
	strcpy((char*)wifi_config.sta.password, wifi_password);
}