#include "serialTask.h"

#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "main.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"

void serialTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_SERIAL;
    
    osDelay(TASK_INIT_DELAY_SERIAL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        // CHECK_STACK_AVAILABLE(serialTask);
        // CHECK_HEAP_AVAILABLE();

        tick += delayTick;
        osDelayUntil(tick);
    }
}
