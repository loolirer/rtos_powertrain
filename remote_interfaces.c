// C includes
#include <stdio.h>
#include <stdint.h>

// Pico 2W includes
#include "lwip/sockets.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <event_groups.h>

// Custom includes
#include "remote_interfaces.h"
#include "wifi_manager.h"

void telemetry_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;
    float telemetry_data[N_MOTORS];

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TELEMETRY_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(TELEMETRY_IP);

    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    printf("[%s] Sending speed measurements to %s:%d\n", TELEMETRY_TASK, TELEMETRY_IP, TELEMETRY_PORT);

    for( ; ; ) {
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

        for (int i = 0; i < N_MOTORS; i++) {
            xQueuePeek(MotorConfig[i].telemetry_queue, &telemetry_data[i], 0U);
        }
        sendto(sock, telemetry_data, sizeof(telemetry_data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setpoint_task(void *pvParameters) {
    MotorConfig_t *motors = (MotorConfig_t *)pvParameters;

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct timeval timeout;
    timeout.tv_sec = TIMEOUT_S;
    timeout.tv_usec = TIMEOUT_US;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    float timeout_setpoint = 0.0f;

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SETPOINT_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    uint8_t buffer[128]; 
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    printf("[%s] Listening for setpoints on port %d\n", SETPOINT_TASK, SETPOINT_PORT);

    for( ; ; ) {
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

        int len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
                           
        if (len > 0) {
            if (len % sizeof(float) != 0) {
                printf("[%s] Ignored corrupted packet: %d bytes is not a clean float array\n", SETPOINT_TASK, len);
                continue; 
            }

            int num_received_setpoints = len / sizeof(float);
            float *received_setpoints = (float *)buffer;

            if (num_received_setpoints == N_MOTORS) {
                
                printf("[%s] ", SETPOINT_TASK);
                for (int i = 0; i < N_MOTORS; i++) {
                    printf("M%d: %.2f | ", i, received_setpoints[i]);
                    
                    xQueueOverwrite(motors[i].setpoint_queue, &received_setpoints[i]);
                }
                printf("\n");

            } else {
                printf("[%s] Size Mismatch! Expected %d floats, got %d\n", SETPOINT_TASK, N_MOTORS, num_received_setpoints);
            }
        } else {
            for (int i = 0; i < N_MOTORS; i++) {
                xQueueOverwrite(motors[i].setpoint_queue, &timeout_setpoint);
            }
            printf("[%s] Timeout! Motor stopped\n", SETPOINT_TASK);
        }
    }
}