#include "stateTask.h"

#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "main.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"
#include "ws2812_driver.h"
#include "oled_iic_4.h"

void stateTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_STATE;
    
    osDelay(TASK_INIT_DELAY_STATE);
    uint32_t tick = osKernelGetTickCount();

    // ws2812_start();
    while(1){
        CHECK_STACK_AVAILABLE(stateTask);
        // ws2812_run();
        if(usr_delay_ms(1000) == USR_DELAY_SUCCESS_END){
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

        tick += delayTick;
        osDelayUntil(tick);
    }
}
