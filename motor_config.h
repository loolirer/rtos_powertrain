#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <queue.h>

#define MOTOR_CONTROLLER_TASK "MOTOR_CONTROLLER"
#define PWM_FREQ 20000
#define CLK_DIV configCPU_CLOCK_HZ / 100000000.0f
#define PWM_WRAP 100000000 / PWM_FREQ
#define N_MOTORS 2
#define MOTOR_ENC_A_PINS   {10, 6}
#define MOTOR_ENC_B_PINS   {9, 5}
#define MOTOR_PWM_FWD_PINS {2, 0}
#define MOTOR_PWM_REV_PINS {3, 1}
#define CTRL_DT_MS         10
#define CTRL_ALPHA         0.25f
#define CTRL_KP            5.0f
#define CTRL_KI            4.0f
#define CTRL_KD            0.0f
#define CTRL_INT_MAX       200.0f
#define TICKS_PER_REV 360.0f 

typedef struct {
    int motor_id;
    float target_speed;
    float measured_speed;
    float alpha;
    float integral_max_effort;
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

extern MotorConfig_t MotorControls[N_MOTORS];

void init_motor_hardware(void);

#endif // MOTOR_CONFIG_H