#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configMAX_PRIORITIES      5
#define configUSE_PREEMPTION      1
#define configUSE_TIME_SLICING    1
#define configUSE_MUTEXES         1
#define configCOUNTING_SEMAPHORES 1

// This example uses a common include to avoid repetition
#include "FreeRTOSConfig_examples_common.h"

#endif
