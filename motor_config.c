// Pico 2W includes
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Custom includes
#include "motor_config.h"
#include "motor_control.h"

void init_motor_hardware() {
    const uint8_t enc_a_pins[N_MOTORS] = MOTOR_ENC_A_PINS;
    const uint8_t enc_b_pins[N_MOTORS] = MOTOR_ENC_B_PINS;
    const uint8_t pwm_fwd_pins[N_MOTORS] = MOTOR_PWM_FWD_PINS;
    const uint8_t pwm_rev_pins[N_MOTORS] = MOTOR_PWM_REV_PINS;
    float dt = (float)CTRL_DT_MS / 1000.0f;
    
    for(int i = 0; i < N_MOTORS; i++) {
        MotorControls[i].motor_id = i;
        MotorControls[i].enc_a_pin = enc_a_pins[i];
        MotorControls[i].enc_b_pin = enc_b_pins[i];
        MotorControls[i].pwm_fwd_pin = pwm_fwd_pins[i];
        MotorControls[i].pwm_rev_pin = pwm_rev_pins[i];
        MotorControls[i].alpha = CTRL_ALPHA;
        MotorControls[i].integral_max_effort = CTRL_INT_MAX;
        MotorControls[i].Kp = CTRL_KP; 
        MotorControls[i].Ki = CTRL_KI;
        MotorControls[i].Kd = CTRL_KD;
        MotorControls[i].dt_ms = CTRL_DT_MS;
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