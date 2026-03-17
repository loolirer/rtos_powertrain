// Pico 2W includes
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <event_groups.h>

// Standard includes
#include <math.h>
#include <stdio.h> 

#define SETPOINT_TASK "WIFI_SETPOINT"
#define TELEMETRY_TASK "WIFI_TELEMETRY"
#define WIFI_MANAGER_TASK "WIFI_MANAGER"
#define WIFI_SSID "Quarto"
#define WIFI_PASSWORD "07055492"
#define SETPOINT_PORT 1234
#define TELEMETRY_PORT 4321
#define TELEMETRY_IP "192.168.1.106"
#define WIFI_CONNECTED (1 << 0)
#define TIMEOUT_US 1000000
TaskHandle_t wifi_manager_handle = NULL;
EventGroupHandle_t wifi_event_group;

#define MOTOR_CONTROLLER_TASK "MOTOR_CONTROLLER"
#define PWM_FREQ 20000
#define CLK_DIV configCPU_CLOCK_HZ / 100000000.0f
#define PWM_WRAP 100000000 / PWM_FREQ
#define N_MOTORS 2
#define TICKS_PER_REV 360.0f 
#define LEFT 0
#define RIGHT 1
typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
    float alpha;
    float Kp;
    float Ki;
    float Kd;
    float dt;
    int dt_ms;
    volatile long encoder_ticks;
    uint enc_a_pin;
    uint enc_b_pin;
    uint pwm_fwd_pin;
    uint pwm_rev_pin;
    uint pwm_slice_fwd;
    uint pwm_slice_rev;
    QueueHandle_t setpoint_queue;
    QueueHandle_t telemetry_queue;
} MotorConfig_t;
MotorConfig_t MotorControls[N_MOTORS];

void encoder_isr(uint gpio, uint32_t events) {
    for (int i = 0; i < N_MOTORS; i++) {
        if (gpio == MotorControls[i].enc_a_pin || gpio == MotorControls[i].enc_b_pin) {
            
            bool a_state = gpio_get(MotorControls[i].enc_a_pin);
            bool b_state = gpio_get(MotorControls[i].enc_b_pin);

            if (gpio == MotorControls[i].enc_a_pin) {
                if (a_state == b_state) {
                    MotorControls[i].encoder_ticks++;
                } else {
                    MotorControls[i].encoder_ticks--;
                }
            } else if (gpio == MotorControls[i].enc_b_pin) {
                if (a_state == b_state) {
                    MotorControls[i].encoder_ticks--;
                } else {
                    MotorControls[i].encoder_ticks++;
                }
            }
            
            break;
        }
    }
}

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
            if(xQueueReceive(MotorConfig[i].telemetry_queue, &telemetry_data[i], 0U)!= pdPASS) {
                telemetry_data[i] = 0.0f;
            }
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
    timeout.tv_sec = 0;
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
                    
                    xQueueSend(motors[i].setpoint_queue, &received_setpoints[i], 0U);
                }
                printf("\n");

            } else {
                printf("[%s] Size Mismatch! Expected %d floats, got %d\n", SETPOINT_TASK, N_MOTORS, num_received_setpoints);
            }
        } else {
            for (int i = 0; i < N_MOTORS; i++) {
                xQueueSend(motors[i].setpoint_queue, &timeout_setpoint, 0U);
            }
            printf("[%s] Timeout! Motor stopped\n", SETPOINT_TASK);
        }
    }
}

void motor_controller_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;
    
    long previous_ticks = 0;
    float received_setpoint = 0.0f;

    float error = 0.0f;
    float previous_error = 0.0f;
    float accumulated_error = 0.0f;
    const float integral_max = 50.0f; 
    const float integral_min = -50.0f;

    const TickType_t xFrequency = pdMS_TO_TICKS(MotorConfig->dt_ms); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    printf("[%s_%d] Initializing Controller...\n", MOTOR_CONTROLLER_TASK, MotorConfig->motor_id);

    for( ; ; ) {
        if(xQueueReceive(MotorConfig->setpoint_queue, &received_setpoint, 0U) == pdPASS) {
            MotorConfig->target_speed = received_setpoint;
        }

        long current_ticks = MotorConfig->encoder_ticks;
        long delta_ticks = current_ticks - previous_ticks;
        previous_ticks = current_ticks;

        float revolutions = (float)delta_ticks / TICKS_PER_REV;
        float raw_speed = (revolutions * 2.0f * M_PI) / MotorConfig->dt;
        MotorConfig->measured_speed = (MotorConfig->alpha * raw_speed) + ((1.0f - MotorConfig->alpha) * MotorConfig->measured_speed);

        xQueueSend(MotorConfig->telemetry_queue, &MotorConfig->measured_speed, 0U);
        
        error = MotorConfig->target_speed - MotorConfig->measured_speed;
        float P = MotorConfig->Kp * error;
        accumulated_error = accumulated_error + (error * MotorConfig->dt);
        if (accumulated_error > integral_max) {
            accumulated_error = integral_max;
        } else if (accumulated_error < integral_min) {
            accumulated_error = integral_min;
        }
        float I = MotorConfig->Ki * accumulated_error;
        float D = MotorConfig->Kd * (error - previous_error) / MotorConfig->dt;
        previous_error = error;
        float control_signal = P + I + D;

        if (control_signal > 100.0f) control_signal = 100.0f;
        if (control_signal < -100.0f) control_signal = -100.0f;

        uint16_t duty_cycle = (uint16_t)(fabs(control_signal) * PWM_WRAP / 100.0f);

        if (control_signal > 0.0f) {
            pwm_set_gpio_level(MotorConfig->pwm_fwd_pin, duty_cycle);
            pwm_set_gpio_level(MotorConfig->pwm_rev_pin, 0);
            
        } else if (control_signal < 0.0f) {
            pwm_set_gpio_level(MotorConfig->pwm_fwd_pin, 0);
            pwm_set_gpio_level(MotorConfig->pwm_rev_pin, duty_cycle);
            
        } else {
            pwm_set_gpio_level(MotorConfig->pwm_fwd_pin, 0);
            pwm_set_gpio_level(MotorConfig->pwm_rev_pin, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void init_motor_hardware() {
    int dt_ms = 10;
    float dt = float(dt_ms) / 1000.0f;
    float alpha = 0.25f;
    float Kp = 5.0f;
    float Ki = 4.0f;
    float Kd = 0.0f;
    
    MotorControls[LEFT].motor_id = LEFT;
    MotorControls[LEFT].enc_a_pin = 10;
    MotorControls[LEFT].enc_b_pin = 9;
    MotorControls[LEFT].pwm_fwd_pin = 2;
    MotorControls[LEFT].pwm_rev_pin = 3;
    MotorControls[LEFT].alpha = alpha;
    MotorControls[LEFT].Kp = Kp; 
    MotorControls[LEFT].Ki = Ki;
    MotorControls[LEFT].Kd = Kd;
    MotorControls[LEFT].dt_ms = dt_ms;
    MotorControls[LEFT].dt = dt;

    MotorControls[RIGHT].motor_id = RIGHT;
    MotorControls[RIGHT].enc_a_pin = 6;
    MotorControls[RIGHT].enc_b_pin = 5;
    MotorControls[RIGHT].pwm_fwd_pin = 0;
    MotorControls[RIGHT].pwm_rev_pin = 1;
    MotorControls[RIGHT].alpha = alpha;
    MotorControls[RIGHT].Kp = Kp; 
    MotorControls[RIGHT].Ki = Ki;
    MotorControls[RIGHT].Kd = Kd;
    MotorControls[RIGHT].dt_ms = dt_ms;
    MotorControls[RIGHT].dt = dt;

    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].setpoint_queue = xQueueCreate(1, sizeof(float));
        MotorControls[i].telemetry_queue = xQueueCreate(1, sizeof(float));
        MotorControls[i].encoder_ticks = 0;

        gpio_init(MotorControls[i].enc_a_pin);
        gpio_set_dir(MotorControls[i].enc_a_pin, GPIO_IN);
        gpio_pull_up(MotorControls[i].enc_a_pin);
        
        gpio_init(MotorControls[i].enc_b_pin);
        gpio_set_dir(MotorControls[i].enc_b_pin, GPIO_IN);
        gpio_pull_up(MotorControls[i].enc_b_pin);

        if (i == 0) {
            gpio_set_irq_enabled_with_callback(
                MotorControls[i].enc_a_pin, 
                GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                true, 
                &encoder_isr
            );
        } else {
            gpio_set_irq_enabled(
                MotorControls[i].enc_a_pin, 
                GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                true
            );
        }

        gpio_set_irq_enabled(MotorControls[i].enc_b_pin, 
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                             true);

        gpio_set_function(MotorControls[i].pwm_fwd_pin, GPIO_FUNC_PWM);
        gpio_set_function(MotorControls[i].pwm_rev_pin, GPIO_FUNC_PWM);

        MotorControls[i].pwm_slice_fwd = pwm_gpio_to_slice_num(MotorControls[i].pwm_fwd_pin);
        MotorControls[i].pwm_slice_rev = pwm_gpio_to_slice_num(MotorControls[i].pwm_rev_pin);

        pwm_set_clkdiv(MotorControls[i].pwm_slice_fwd, CLK_DIV);
        pwm_set_clkdiv(MotorControls[i].pwm_slice_rev, CLK_DIV);

        pwm_set_wrap(MotorControls[i].pwm_slice_fwd, PWM_WRAP);
        pwm_set_wrap(MotorControls[i].pwm_slice_rev, PWM_WRAP);

        pwm_set_gpio_level(MotorControls[i].pwm_fwd_pin, 0);
        pwm_set_gpio_level(MotorControls[i].pwm_rev_pin, 0);

        pwm_set_enabled(MotorControls[i].pwm_slice_fwd, true);
        pwm_set_enabled(MotorControls[i].pwm_slice_rev, true);
    }
}

int main() {
    stdio_init_all(); // From Pico stdlib
    
    init_motor_hardware();
    xTaskCreate(motor_controller_task, MOTOR_CONTROLLER_TASK, 512, &MotorControls[LEFT], 4, NULL);
    xTaskCreate(motor_controller_task, MOTOR_CONTROLLER_TASK, 512, &MotorControls[RIGHT], 4, NULL);
    wifi_event_group = xEventGroupCreate();
    xTaskCreate(wifi_manager_task, WIFI_MANAGER_TASK, 1024, NULL, 3, &wifi_manager_handle);
    xTaskCreate(setpoint_task, SETPOINT_TASK, 1024, (void*)MotorControls, 2, NULL);
    xTaskCreate(telemetry_task, TELEMETRY_TASK, 1024, (void*)MotorControls, 1, NULL);

    vTaskStartScheduler();

    for( ; ; ) {}

    return 0;
}