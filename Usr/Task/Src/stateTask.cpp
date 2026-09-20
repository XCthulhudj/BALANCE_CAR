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
    // oled_show_picture(0, 0);
    oled_show_fonts(0, 0);
    while(1){
        ws2812_rainbow(1);
        // oled_show_anim(0, 100);

        tick += delayTick;
        osDelayUntil(tick);
    }
}
