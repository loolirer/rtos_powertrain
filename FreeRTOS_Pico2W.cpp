// Pico 2W includes
#include "pico/stdlib.h"
#include "hardware/pwm.h"

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

#define SETPOINT_TASK "WIFI_SETPOINT"
#define WIFI_SSID "Quarto"
#define WIFI_PASSWORD "07055492"
#define PORT 1234

#define MOTOR_CONTROLLER_TASK "MOTOR_CONTROLLER"
#define N_MOTORS 2
typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
    float Kp;
    float Ki;
    float Kd;
    float dt;
    int dt_ms;
    QueueHandle_t encoder_queue;
    QueueHandle_t setpoint_queue;
} MotorConfig_t;
MotorConfig_t MotorControls[N_MOTORS];

#define ENCODER_TASK "ENCODER"

// Receives UDP string, parses to floats, and routes desired 
// angular velocity to the specific motor's queue
void setpoint_task(void *pvParameters) {
    MotorConfig_t *motors = (MotorConfig_t *)pvParameters;
    float phi_dots[N_MOTORS];

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

    uint8_t buffer[128]; 
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    printf("[%s] Listening for setpoints on UDP port %d...\n", SETPOINT_TASK, PORT);

    for ( ; ; ) {
        int len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
                           
        if (len > 0) {
            if (len % sizeof(float) != 0) {
                printf("[%s] Ignored corrupted packet: %d bytes is not a clean float array.\n", SETPOINT_TASK, len);
                continue; 
            }

            int num_received_setpoints = len / sizeof(float);
            float *received_setpoints = (float *)buffer;

            if (num_received_setpoints == N_MOTORS) {
                
                printf("[%s]", SETPOINT_TASK);
                for (int i = 0; i < N_MOTORS; i++) {
                    printf("M%d: %.2f | ", i, received_setpoints[i]);
                    
                    xQueueSend(motors[i].setpoint_queue, &received_setpoints[i], 0U);
                }
                printf("\n");

            } else {
                printf("[%s] Size Mismatch! Expected %d floats, got %d.\n", SETPOINT_TASK, N_MOTORS, num_received_setpoints);
            }
        }
    }
}

// Actuate on motor
void motor_controller_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;

    float received_setpoint = 0.0f;

    float error = 0.0f;
    float previous_error = 0.0f;
    float accumulated_error = 0.0f;
    const float integral_max = 50.0f; 
    const float interal_min = -50.0f;

    const TickType_t xFrequency = pdMS_TO_TICKS(MotorConfig->dt_ms); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for( ; ; ) {
        if(xQueueReceive(MotorConfig->setpoint_queue, &received_setpoint, 0U) == pdPASS) {
            MotorConfig->target_speed = received_setpoint;
        }
        xQueueReceive(MotorConfig->encoder_queue, &MotorConfig->measured_speed, 0U);

        error = MotorConfig->target_speed - MotorConfig->measured_speed;
        float P = MotorConfig->Kp * error;
        accumulated_error = accumulated_error + (error * MotorConfig->dt);
        if (accumulated_error > integral_max) {
            accumulated_error = integral_max;
        } else if (accumulated_error < interal_min) {
            accumulated_error = interal_min;
        }
        float I = MotorConfig->Ki * accumulated_error;
        float D = MotorConfig->Kd * (error - previous_error) / MotorConfig->dt;
        previous_error = error;
        float control_signal = P + I + D;

        if (control_signal > 100.0f) control_signal = 100.0f;
        if (control_signal < -100.0f) control_signal = -100.0f;

        //TODO: PWM LOGIC
        
        printf("[%s_%d] Target: %.2f | Measured: %.2f | PWM Output: %.2f\n", 
               MOTOR_CONTROLLER_TASK, MotorConfig->motor_id, 
               MotorConfig->target_speed, MotorConfig->measured_speed, control_signal);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Reads encoder and sends the angular velocity 
// of the wheel in rad/s
void encoder_task(void *pvParameters) {
    float phi_dot = 0.0f;
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;

    for( ; ; ) {
        xQueueSend(MotorConfig->encoder_queue, &phi_dot, 0U);
        vTaskDelay(100);
    }
}

int main() {
    stdio_init_all(); // From Pico stdlib

    xTaskCreate(setpoint_task, SETPOINT_TASK, 1024, (void*)MotorControls, 2, NULL);
    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].motor_id = i;
        MotorControls[i].target_speed = 0.0f;
        MotorControls[i].measured_speed = 0.0f;
        MotorControls[i].Kp = 5.0f;
        MotorControls[i].Ki = 2.0f;
        MotorControls[i].Kd = 0.0f;
        MotorControls[i].dt_ms = 10;
        MotorControls[i].dt = float(MotorControls[i].dt_ms) / 1000.0f;
        MotorControls[i].setpoint_queue = xQueueCreate(1, sizeof(float));
        MotorControls[i].encoder_queue = xQueueCreate(1, sizeof(float));
        xTaskCreate(encoder_task, "ENCODER", 256, &MotorControls[i], 1, NULL);
        xTaskCreate(motor_controller_task, "MOTOR", 256, &MotorControls[i], 2, NULL);
    }

    vTaskStartScheduler();

    for( ; ; ) {}

    return 0;
}