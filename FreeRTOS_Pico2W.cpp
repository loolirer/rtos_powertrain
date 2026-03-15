// Pico 2W includes
#include "pico/stdlib.h"

// FreeRTOS includes
extern "C" {
    #include "pico/cyw43_arch.h"
    #include "lwip/sockets.h"
    #include <FreeRTOS.h>
    #include <task.h>
    #include <queue.h>
    #include <timers.h>
    #include <semphr.h>
}

// Standard includes
#include <stdio.h>

#define SETPOINT_TASK "SETPOINT_TASK"
#define WIFI_SSID "Quarto"
#define WIFI_PASSWORD "07055492"
#define PORT 1234
static QueueHandle_t setpoint_queue;

#define MOTOR_TASK "MOTOR_TASK"
#define N_MOTORS 2
typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
    QueueHandle_t encoder_queue;
} MotorConfig_t;
MotorConfig_t MotorControls[N_MOTORS];

#define ENCODER_TASK "ENCODER_TASK"

// Receives UDP string, parses to floats, and sends desired 
// angular velocity of each wheel (in rad/s) to the motor queue
void setpoint_task(void *pvParameters) {
    QueueHandle_t setpoint_queue = (QueueHandle_t)pvParameters;
    float phi_dots[2] = {0.0f, 0.0f};

    if (cyw43_arch_init()) {
        printf("[%s] WiFi init failed\n", SETPOINT_TASK);
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK)) {
        printf("[%s] Failed to connect.\n", SETPOINT_TASK);
        vTaskDelete(NULL);
    }
    printf("[%s] Connected! IP: %s\n", SETPOINT_TASK, ip4addr_ntoa(netif_ip4_addr(netif_default)));

    struct sockaddr_in server_addr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0); 
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    char buffer[128];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    printf("[%s] Listening for setpoints on UDP port %d...\n", SETPOINT_TASK, PORT);

    for ( ; ; ) {
        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                           (struct sockaddr *)&client_addr, &addr_len);
                           
        if (len > 0) {
            buffer[len] = '\0';
            
            int parsed = sscanf(buffer, "%f %f", &phi_dots[0], &phi_dots[1]);
            if (parsed == 2) {
                printf("[%s] Motor 0: %.2f rad/s | Motor 1: %.2f rad/s\n", SETPOINT_TASK, phi_dots[0], phi_dots[1]);
                xQueueSend(setpoint_queue, &phi_dots, 0U);

            } else {
                printf("[%s] Ignored invalid UDP packet: '%s'.\n", SETPOINT_TASK, buffer);
            }
        }
    }
}

// Actuate on motor
void motor_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;
    float setpoints[N_MOTORS];

    for( ; ; ) {
        if(xQueueReceive(setpoint_queue, &setpoints, 0U) == pdPASS) {
            MotorConfig->target_speed = setpoints[MotorConfig->motor_id];
        }

        xQueueReceive(MotorConfig->encoder_queue, &MotorConfig->measured_speed, 0U);
        printf(
            "[MOTOR_%d]: Trying %f rad/s | Actually at %f rad/s\n", 
            MotorConfig->motor_id, 
            MotorConfig->target_speed,
            MotorConfig->measured_speed
        );
        vTaskDelay(500);
    }
}

// Reads encoder and sends the angular velocity 
// of the wheel in rad/s
void encoder_task(void *pvParameters) {
    float phi_dot = 0.0;
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;

    for( ; ; ) {
        xQueueSend(MotorConfig->encoder_queue, &phi_dot, 0U);
        vTaskDelay(100);
    }
}

int main() {
    stdio_init_all(); // From Pico stdlib
    
    setpoint_queue = xQueueCreate(1, sizeof(float[2]));
    xTaskCreate(setpoint_task, SETPOINT_TASK, 1024, (void*)setpoint_queue, 2, NULL);

    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].motor_id = i;
        MotorControls[i].target_speed = 0.0;
        MotorControls[i].measured_speed = 0.0;
        MotorControls[i].encoder_queue = xQueueCreate(1, sizeof(float));
        xTaskCreate(encoder_task, "ENCODER", 256, &MotorControls[i], 1, NULL);
        xTaskCreate(motor_task, "MOTOR", 256, &MotorControls[i], 2, NULL);
    }

    vTaskStartScheduler();

    for( ; ; ) {}

    return 0;
}