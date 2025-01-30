#ifndef DRIVERS_WIFI_H
#define DRIVERS_WIFI_H

void wifi_set_name(char *name);
char* wifi_get_name();
void wifi_set_password(char *pwd);
char* wifi_get_password();

void init_wifi();
#endif