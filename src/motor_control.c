// C includes
#include <stdio.h>
#include <math.h>

// Pico 2W includes
#include "hardware/pwm.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <queue.h>

// Custom includes
#include "motor_control.h"

void motor_controller_task(void *pvParameters) {
    MotorConfig_t *MotorConfig = (MotorConfig_t *)pvParameters;
    
    long previous_ticks = 0;
    float received_setpoint = 0.0f;

    float error = 0.0f;
    float previous_error = 0.0f;
    float accumulated_error = 0.0f;
    float integral_max = 0.0f;

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

        xQueueOverwrite(MotorConfig->telemetry_queue, &MotorConfig->measured_speed);
        
        error = MotorConfig->target_speed - MotorConfig->measured_speed;
        float P = MotorConfig->Kp * error;
        if (MotorConfig->Ki > 0.001f) { 
            accumulated_error = accumulated_error + (error * MotorConfig->dt);
            integral_max = MotorConfig->integral_max_effort / MotorConfig->Ki;
            
            if (accumulated_error > integral_max) {
                accumulated_error = integral_max;
            } else if (accumulated_error < -integral_max) {
                accumulated_error = -integral_max;
            }
        } else {
            accumulated_error = 0.0f; 
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