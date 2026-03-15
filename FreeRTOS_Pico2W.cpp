// Pico 2W includes
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

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
#define TICKS_PER_REV 180.0f 
#define PI 3.1415926535f
typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
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
} MotorConfig_t;
MotorConfig_t MotorControls[N_MOTORS];

void encoder_isr(uint gpio, uint32_t events) {
    for (int i = 0; i < N_MOTORS; i++) {
        if (gpio == MotorControls[i].enc_a_pin) {
            
            if (gpio_get(MotorControls[i].enc_a_pin) == gpio_get(MotorControls[i].enc_b_pin)) {
                MotorControls[i].encoder_ticks--;
            } else {
                MotorControls[i].encoder_ticks++;
            }
            
            break;
        }
    }
}

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

// Reads the encoder, calculates speed, and actuates the motor
void motor_controller_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;
    
    long previous_ticks = 0;
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

        long current_ticks = MotorConfig->encoder_ticks;
        long delta_ticks = current_ticks - previous_ticks;
        previous_ticks = current_ticks;

        float revolutions = (float)delta_ticks / TICKS_PER_REV;
        MotorConfig->measured_speed = (revolutions * 2.0f * PI) / MotorConfig->dt;

        if (MotorConfig->motor_id == 0) {
            printf("[%s_%d] Speed: %f \n", 
               MOTOR_CONTROLLER_TASK, MotorConfig->motor_id, 
               MotorConfig->measured_speed);
        }
        
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
        //printf("[%s_%d] Target: %.2f | Measured: %.2f | PWM Output: %.2f\n", 
        //       MOTOR_CONTROLLER_TASK, MotorConfig->motor_id, 
        //       MotorConfig->target_speed, MotorConfig->measured_speed, control_signal);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void init_motor_hardware() {
    int dt_ms = 10;
    float dt = float(dt_ms) / 1000.0f;

    MotorControls[0].motor_id = 0;
    MotorControls[0].enc_a_pin = 6;
    MotorControls[0].enc_b_pin = 5;
    MotorControls[0].pwm_fwd_pin = 0;
    MotorControls[0].pwm_rev_pin = 1;
    MotorControls[0].Kp = 1.5f; 
    MotorControls[0].Ki = 0.1f;
    MotorControls[0].Kd = 0.0f;
    MotorControls[0].dt_ms = dt_ms;
    MotorControls[0].dt = dt;

    MotorControls[1].motor_id = 1;
    MotorControls[1].enc_a_pin = 10;
    MotorControls[1].enc_b_pin = 9;
    MotorControls[1].pwm_fwd_pin = 2;
    MotorControls[1].pwm_rev_pin = 3;
    MotorControls[1].Kp = 1.5f; 
    MotorControls[1].Ki = 0.1f;
    MotorControls[1].Kd = 0.0f;
    MotorControls[1].dt_ms = dt_ms;
    MotorControls[1].dt = dt;

    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].setpoint_queue = xQueueCreate(1, sizeof(float));
        MotorControls[i].encoder_ticks = 0;

        gpio_init(MotorControls[i].enc_a_pin);
        gpio_set_dir(MotorControls[i].enc_a_pin, GPIO_IN);
        gpio_pull_up(MotorControls[i].enc_a_pin);
        
        gpio_init(MotorControls[i].enc_b_pin);
        gpio_set_dir(MotorControls[i].enc_b_pin, GPIO_IN);
        gpio_pull_up(MotorControls[i].enc_b_pin);

        if (i == 0) {
            gpio_set_irq_enabled_with_callback(MotorControls[i].enc_a_pin, 
                                               GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                               true, &encoder_isr);
        } else {
            gpio_set_irq_enabled(MotorControls[i].enc_a_pin, 
                                 GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                 true);
        }

        gpio_set_function(MotorControls[i].pwm_fwd_pin, GPIO_FUNC_PWM);
        gpio_set_function(MotorControls[i].pwm_rev_pin, GPIO_FUNC_PWM);

        MotorControls[i].pwm_slice_fwd = pwm_gpio_to_slice_num(MotorControls[i].pwm_fwd_pin);
        MotorControls[i].pwm_slice_rev = pwm_gpio_to_slice_num(MotorControls[i].pwm_rev_pin);

        pwm_set_clkdiv(MotorControls[i].pwm_slice_fwd, 1.25f);
        pwm_set_wrap(MotorControls[i].pwm_slice_fwd, 10000);
        
        pwm_set_clkdiv(MotorControls[i].pwm_slice_rev, 1.25f);
        pwm_set_wrap(MotorControls[i].pwm_slice_rev, 10000);

        pwm_set_gpio_level(MotorControls[i].pwm_fwd_pin, 0);
        pwm_set_gpio_level(MotorControls[i].pwm_rev_pin, 0);

        pwm_set_enabled(MotorControls[i].pwm_slice_fwd, true);
        pwm_set_enabled(MotorControls[i].pwm_slice_rev, true);

        xTaskCreate(motor_controller_task, "MOTOR", 512, &MotorControls[i], 2, NULL);
    }
}

int main() {
    stdio_init_all(); // From Pico stdlib

    xTaskCreate(setpoint_task, SETPOINT_TASK, 1024, (void*)MotorControls, 2, NULL);
    init_motor_hardware();

    vTaskStartScheduler();

    for( ; ; ) {}

    return 0;
}