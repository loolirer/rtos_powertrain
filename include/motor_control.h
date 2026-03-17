#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include "motor_config.h"

#define MOTOR_CONTROLLER_TASK "MOTOR_CONTROLLER"

void motor_controller_task(void *pvParameters);

#endif // MOTOR_CONTROL_H