// Pico 2W includes
#include "pico/stdlib.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

// Standard includes
#include <stdio.h>

#define N_MOTORS 2

typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
    QueueHandle_t encoder_queue;
} MotorConfig_t;
MotorConfig_t MotorControls[N_MOTORS];

static QueueHandle_t setpoint_queue;

// Actuacte on motor
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

// Receives setpoint message and sends the desired 
// angular velocity of each wheel in rad/s
void setpoint_task(void *pvParameters) {
    float phi_dots[] = {0.0, 0.0};
    QueueHandle_t setpoint_queue = (QueueHandle_t)pvParameters;

    for( ; ; ) {
        xQueueSend(setpoint_queue, &phi_dots, 0U);
        vTaskDelay(100);
    }
}

int main() {
    stdio_init_all(); // From Pico stdlib
    
    setpoint_queue = xQueueCreate(1, sizeof(float[2]));
    xTaskCreate(setpoint_task, "SETPOINT", 256, (void*)setpoint_queue, 1, NULL);
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