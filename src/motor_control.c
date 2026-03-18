// C includes
#include <stdio.h>
#include <math.h>

// Pico 2W includes
#include "hardware/pwm.h"
#include "hardware/timer.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <queue.h>

// Custom includes
#include "motor_control.h"

static struct repeating_timer motor_control_timer;

static bool motor_control_timer_callback(struct repeating_timer *t) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    float received_setpoint = 0.0f;

    for (int i = 0; i < N_MOTORS; i++) {
        MotorConfig_t *MotorConfig = &MotorControls[i];

        if(xQueueReceiveFromISR(MotorConfig->setpoint_queue, &received_setpoint, &xHigherPriorityTaskWoken) == pdPASS) {
            MotorConfig->target_speed = received_setpoint;
        }

        long current_ticks = MotorConfig->encoder_ticks;
        long delta_ticks = current_ticks - MotorConfig->last_ticks; 
        MotorConfig->last_ticks = current_ticks;

        float revolutions = (float)delta_ticks / TICKS_PER_REV;
        float raw_speed = (revolutions * 2.0f * M_PI) / MotorConfig->dt;
        MotorConfig->measured_speed = (MotorConfig->alpha * raw_speed) + ((1.0f - MotorConfig->alpha) * MotorConfig->measured_speed);
        
        float current_speed = MotorConfig->measured_speed;
        xQueueOverwriteFromISR(MotorConfig->telemetry_queue, &current_speed, &xHigherPriorityTaskWoken);
        
        float error = MotorConfig->target_speed - MotorConfig->measured_speed;
        float P = MotorConfig->Kp * error;
        if (MotorConfig->Ki > 0.001f) { 
            MotorConfig->accumulated_error += (error * MotorConfig->dt); 
            float integral_max = MotorConfig->integral_max_effort / MotorConfig->Ki;
            
            if (MotorConfig->accumulated_error > integral_max) {
                MotorConfig->accumulated_error = integral_max;
            } else if (MotorConfig->accumulated_error < -integral_max) {
                MotorConfig->accumulated_error = -integral_max;
            }
        } else {
            MotorConfig->accumulated_error = 0.0f; 
        }
        float I = MotorConfig->Ki * MotorConfig->accumulated_error;
        float D = MotorConfig->Kd * (error - MotorConfig->previous_error) / MotorConfig->dt; 
        MotorConfig->previous_error = error;
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
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true; 
}

void start_motor_control_timer(void) {
    printf("[%s] Starting hardware timer...\n", MOTOR_CONTROL_TIMER, CTRL_DT_MS);
    add_repeating_timer_ms(-CTRL_DT_MS, motor_control_timer_callback, NULL, &motor_control_timer);
}