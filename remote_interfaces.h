#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <FreeRTOS.h>
#include <task.h>
#include "motor_config.h"

#define SETPOINT_TASK "WIFI_SETPOINT"
#define TELEMETRY_TASK "WIFI_TELEMETRY"
#define SETPOINT_PORT 1234
#define TELEMETRY_PORT 4321
#define TELEMETRY_IP "192.168.1.106"
#define TIMEOUT_S  1
#define TIMEOUT_US 0

void setpoint_task(void *pvParameters);
void telemetry_task(void *pvParameters);

#endif // NETWORK_UTILS_H