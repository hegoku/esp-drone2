#include <string.h>
#include "drivers/wifi.h"
#include "misc/config.h"

static char wifi_name[20];
static char wifi_password[30];

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
}