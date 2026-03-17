// Pico 2W Includes
#include "pico/stdlib.h"

// RTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>

// Custom includes
#include "motor_config.h"
#include "wifi_manager.h"
#include "remote_interfaces.h"
#include "motor_control.h"

MotorConfig_t MotorControls[N_MOTORS];
TaskHandle_t wifi_manager_handle = NULL;
EventGroupHandle_t wifi_event_group;

int main() {
    stdio_init_all();
    init_motor_hardware();

    wifi_event_group = xEventGroupCreate();
    xTaskCreate(motor_controller_task, MOTOR_CONTROLLER_TASK, 512, &MotorControls[LEFT], 4, NULL);
    xTaskCreate(motor_controller_task, MOTOR_CONTROLLER_TASK, 512, &MotorControls[RIGHT], 4, NULL);
    xTaskCreate(wifi_manager_task, WIFI_MANAGER_TASK, 1024, NULL, 3, &wifi_manager_handle);
    xTaskCreate(setpoint_task, SETPOINT_TASK, 1024, (void*)MotorControls, 2, NULL);
    xTaskCreate(telemetry_task, TELEMETRY_TASK, 1024, (void*)MotorControls, 1, NULL);

    vTaskStartScheduler();

    for( ; ; ) {}

    return 0;
}