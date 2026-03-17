#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>
#include "lwip/netif.h"

#define WIFI_MANAGER_TASK "WIFI_MANAGER"
#define WIFI_SSID "Quarto"
#define WIFI_PASSWORD "07055492"
#define WIFI_CONNECTED (1 << 0)

extern TaskHandle_t wifi_manager_handle;
extern EventGroupHandle_t wifi_event_group;

void netif_link_callback(struct netif *netif);
void wifi_manager_task(void *pvParameters);

#endif // WIFI_MANAGER_H