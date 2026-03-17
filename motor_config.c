// Pico 2W includes
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Custom includes
#include "motor_config.h"
#include "motor_control.h"

void init_motor_hardware() {
    MotorControls[LEFT].enc_a_pin = 10;
    MotorControls[LEFT].enc_b_pin = 9;
    MotorControls[LEFT].pwm_fwd_pin = 2;
    MotorControls[LEFT].pwm_rev_pin = 3;

    MotorControls[RIGHT].enc_a_pin = 6;
    MotorControls[RIGHT].enc_b_pin = 5;
    MotorControls[RIGHT].pwm_fwd_pin = 0;
    MotorControls[RIGHT].pwm_rev_pin = 1;

    int dt_ms = 10;
    float dt = (float)dt_ms / 1000.0f;
    float alpha = 0.25f;
    float Kp = 5.0f;
    float Ki = 4.0f;
    float Kd = 0.0f;
    float integral_max_effort = 200.0f;

    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].motor_id = i;
        MotorControls[i].alpha = alpha;
        MotorControls[i].integral_max_effort = integral_max_effort;
        MotorControls[i].Kp = Kp; 
        MotorControls[i].Ki = Ki;
        MotorControls[i].Kd = Kd;
        MotorControls[i].dt_ms = dt_ms;
        MotorControls[i].dt = dt;
        MotorControls[i].encoder_ticks = 0;

        MotorControls[i].setpoint_queue = xQueueCreate(1, sizeof(float));
        MotorControls[i].telemetry_queue = xQueueCreate(1, sizeof(float));

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

        gpio_set_irq_enabled(
            MotorControls[i].enc_b_pin, 
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
            true
        );

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