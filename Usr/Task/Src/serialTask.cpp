#include "serialTask.h"

#include <cmsis_os2.h>
#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"

#include "usr_config.h"
#include "usr_delay.h"
#include "app_debug.h"

void serialTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_SERIAL;
    osDelay(TASK_INIT_DELAY_SERIAL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        DEBUG_PRINT("hello\n");

        tick += delayTick;
        osDelayUntil(tick);
    }
}
