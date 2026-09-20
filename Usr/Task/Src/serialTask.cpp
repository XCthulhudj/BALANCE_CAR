#include "serialTask.h"

#include <cmsis_os2.h>
#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"

#include "usr_config.h"
#include "bsp_uart.h"
#include "usr_delay.h"

void serialTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_SERIAL;
    osDelay(TASK_INIT_DELAY_SERIAL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        // uart1_write_arr((uint8_t*)"fuck", 5);
        if(usr_delay_ms(500) == USR_DELAY_SUCCESS_END){
            uart2_write_arr((uint8_t*)"hello\n", 7);
        }
        
        tick += delayTick;
        osDelayUntil(tick);
    }
}
