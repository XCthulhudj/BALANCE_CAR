#include "motionCtrlTask.h"

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f1xx_hal.h"
#include "iwdg.h"

#include "usr_config.h"

void motionCtrlTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_MOTIONCTRL;
    osDelay(TASK_INIT_DELAY_MOTIONCTRL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        HAL_IWDG_Refresh(&hiwdg);
        
        tick += delayTick;
        osDelayUntil(tick);
    }
}
