#include <string.h>
#include <errno.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "platform/drivers/wifi.h"
#include "misc/config.h"

static const char *TAG = "Wifi";
static const int CONNECTED_BIT = BIT0;

static char wifi_name[32];
static char wifi_password[64];
static unsigned short udp_port;

static wifi_config_t wifi_config;
static wifi_ap_record_t wifidata;

static int is_wifi_init = 0;
static int is_udp_init = 0;

static struct sockaddr_storage source_addr;
static struct sockaddr_in addr_board;
static struct sockaddr_in peer_addr;
static int peer_valid = 0;

static int sock;

static EventGroupHandle_t wifi_event_group;
static esp_netif_t *sta_netif = NULL;

static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;

static void (*upd_revc_handler)(unsigned char *data, int len);

static char rx_buffer[128];
static struct wifi_pack inPack;

static TaskHandle_t wifi_tx_task_handle = NULL;
static esp_timer_handle_t wifi_tx_timer = NULL;

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

void wifi_set_udp_port(unsigned short port)
{
	udp_port = port;
	config_write_ushort("wifi_udp_port", udp_port);
}

unsigned short* wifi_get_udp_port()
{
	return &udp_port;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		esp_netif_ip_info_t ip;
		memset(&ip, 0, sizeof(esp_netif_ip_info_t));
		if (esp_netif_get_ip_info(sta_netif, &ip) == 0) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
			is_wifi_init = 1;
		}
	}
}

static void udp_server_task(void *pvParameters)
{
    static char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
	socklen_t socklen = sizeof(source_addr);

	while(is_wifi_init==0) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	for (;;)
	{
		struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
		dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr_ip4->sin_family = AF_INET;
		dest_addr_ip4->sin_port = htons(udp_port);
		ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

		const int opt=-1;
		int nb = 0;
		nb = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt)); //设置套接字类型
		if(nb==-1) {
			ESP_LOGE(TAG, "set socket error...");
		}

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", udp_port);

		is_udp_init = 1;

		for (;;)
		{
			int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
				inPack.size = len;
				memcpy(inPack.data, rx_buffer, len);
				xQueueSend(rxQueue, &inPack, pdMS_TO_TICKS(2));
				// Get the sender's ip address as string
				memcpy(&peer_addr, &source_addr, sizeof(source_addr));
				peer_valid = 1;
				if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
            }
		}

		if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
			is_udp_init = 0;
			shutdown(sock, 0);
			close(sock);
        }
	}
	vTaskDelete(NULL);
}

void wifi_rx_task(void *param)
{
	static struct wifi_pack p;
	for (;;)
	{
		if (xQueueReceive(rxQueue, &p, portMAX_DELAY) == pdTRUE) {
			if (upd_revc_handler!=0) {
				upd_revc_handler(p.data, p.size);
			}
		}
	}
}

void wifi_tx_task(void *param)
{
	for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        wifi_flush();
    }
}

static void wifi_tx_timer_cb(void *arg)
{
    if (wifi_tx_task_handle != NULL) {
        xTaskNotifyGive(wifi_tx_task_handle);
    }
}

int init_wifi()
{
	upd_revc_handler = 0;
	config_read_string("wifi_name", wifi_name);
	config_read_string("wifi_pwd", wifi_password);
	config_read_ushort("wifi_udp_port", &udp_port);
	if (strlen(wifi_name)==0 || strlen(wifi_password)==0 || udp_port==0)
		return -1;
	strcpy((char *)wifi_config.sta.ssid, wifi_name);
	strcpy((char*)wifi_config.sta.password, wifi_password);

	addr_board.sin_len = 6;
	addr_board.sin_family = AF_INET;
	addr_board.sin_addr.s_addr=htonl(INADDR_BROADCAST);
	addr_board.sin_port=htons(udp_port);

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );

	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

	txQueue = xQueueCreate(150, sizeof(struct wifi_pack));
	rxQueue = xQueueCreate(10, sizeof(struct wifi_pack));

	xTaskCreate(udp_server_task, "udp_server", 1024*3, (void*)AF_INET, 7, NULL);

	xTaskCreatePinnedToCore(wifi_tx_task, "wifi-tx", 1024*3, NULL, 5,
                        &wifi_tx_task_handle, 0);

	const esp_timer_create_args_t wifi_tx_timer_args = {
		.callback = &wifi_tx_timer_cb,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "wifi_tx_timer"
	};

	ESP_ERROR_CHECK(esp_timer_create(&wifi_tx_timer_args, &wifi_tx_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(wifi_tx_timer, 5000));

	xTaskCreate(wifi_rx_task, "wifi-rx", 2048, NULL, 6, NULL);
	return 0;
}

short int wifi_get_rssi()
{
	if (esp_wifi_sta_get_ap_info(&wifidata) == 0)
	{
		return wifidata.rssi;
	}
	return -999;
}

void wifi_set_recv_handler(void (*handler)(unsigned char *data, int len))
{
	upd_revc_handler = handler;
}

void wifi_send(unsigned char *data, int len)
{
    if (data == NULL || len <= 0 || txQueue == NULL) {
        return;
    }

    if (len > WIFI_PACK_MAX_LEN) {
        return;
    }

    struct wifi_pack p;
    p.size = len;
    memcpy(p.data, data, len);

    if (xQueueSend(txQueue, &p, 0) != pdTRUE) {
		ESP_LOGW(TAG, "txQueue full");
        // When TX queue is full, then drop old data and put the new one
        struct wifi_pack old;
        xQueueReceive(txQueue, &old, 0);
        xQueueSend(txQueue, &p, 0);
    }
}

#define WIFI_TX_DATAGRAM_MAX 1200
static unsigned char out[WIFI_TX_DATAGRAM_MAX];

int wifi_flush(void)
{
    if (!is_udp_init || sock < 0 || txQueue == NULL) {
        return -1;
    }
    
    int out_len = 0;
    struct wifi_pack p;

    while (xQueuePeek(txQueue, &p, 0) == pdTRUE) {
        if (p.size <= 0 || p.size > WIFI_PACK_MAX_LEN) {
            xQueueReceive(txQueue, &p, 0);
            continue;
        }

        if (p.size > WIFI_TX_DATAGRAM_MAX) {
            xQueueReceive(txQueue, &p, 0);
            continue;
        }

        if (out_len > 0 && out_len + p.size > WIFI_TX_DATAGRAM_MAX) {
            break;
        }

        xQueueReceive(txQueue, &p, 0);

        memcpy(out + out_len, p.data, p.size);
        out_len += p.size;

        if (out_len >= WIFI_TX_DATAGRAM_MAX) {
            break;
        }
    }

    if (out_len == 0) {
        return 0;
    }

	int ret = 0;
	if (peer_valid) {
		ret = sendto(sock, out, out_len, 0,
                     (struct sockaddr *)&peer_addr,
                     sizeof(peer_addr));
	} else {
		ret = sendto(sock, out, out_len, 0,
                     (struct sockaddr *)&addr_board,
                     sizeof(addr_board));
	}
    
	if (ret<0) {
		int err = errno;
		ESP_LOGW(TAG, "%d %s", err, strerror(err));
	}
    return ret >= 0 ? 0 : ret;
}