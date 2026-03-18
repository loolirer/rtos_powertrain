#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include "motor_config.h"

#define MOTOR_CONTROL_TIMER "MOTOR_CONTROL"

void start_motor_control_timer(void);

#endif // MOTOR_CONTROL_H