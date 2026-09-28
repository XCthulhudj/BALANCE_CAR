#include "serialTask.h"

#include <cmsis_os2.h>
#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"
#include "mpu6050_iic.h"

void serialTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_SERIAL;
    app_debug_init();
    osDelay(TASK_INIT_DELAY_SERIAL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        // CHECK_STACK_AVAILABLE(serialTask);
        // CHECK_HEAP_AVAILABLE();

        tick += delayTick;
        osDelayUntil(tick);
    }
}
