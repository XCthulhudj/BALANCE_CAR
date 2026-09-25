#include "stateTask.h"

#include "usr_config.h"
#include "ws2812_driver.h"
#include "oled_iic_4.h"

void stateTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_STATE;
    osDelay(TASK_INIT_DELAY_STATE);
    uint32_t tick = osKernelGetTickCount();

    //oled
    oled_init();
    ws2812_start();
    while(1){
        
        tick += delayTick;
        osDelayUntil(tick);
    }
}
