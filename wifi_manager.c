// C includes
#include <stdio.h>

// Pico 2W includes
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>

// Custom includes
#include "wifi_manager.h"

void netif_link_callback(struct netif *netif) {
    if (!netif_is_link_up(netif)) {
        if (wifi_manager_handle != NULL) {
            xTaskNotifyGive(wifi_manager_handle);
        }
    }
}

void wifi_manager_task(void *pvParameters) {
    printf("[%s] Initializing WiFi chip...\n", WIFI_MANAGER_TASK);

    if (cyw43_arch_init()) {
        printf("[%s] WiFi init failed! System halted\n", WIFI_MANAGER_TASK);
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();

    netif_set_link_callback(netif_default, netif_link_callback);

    for( ; ; ) {
        printf("[%s] Connecting to SSID: %s...\n", WIFI_MANAGER_TASK, WIFI_SSID);

        if (cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK) == 0) {
            printf("[%s] Connected! IP: %s\n", WIFI_MANAGER_TASK, ip4addr_ntoa(netif_ip4_addr(netif_default)));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            printf("[%s] WiFi connection lost! Preparing to reconnect...\n", WIFI_MANAGER_TASK);
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED);
            
        } else {
            printf("[%s] Connection failed. Retrying in 3 seconds...\n", WIFI_MANAGER_TASK);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}