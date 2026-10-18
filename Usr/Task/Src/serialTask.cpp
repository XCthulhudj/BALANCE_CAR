#include "serialTask.h"

#include <cmsis_os2.h>
#include <cstdint>
#include "FreeRTOS.h"
#include "main.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"

extern UART_HandleTypeDef huart2;

void serialTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_SERIAL;
    
    osDelay(TASK_INIT_DELAY_SERIAL);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        // CHECK_STACK_AVAILABLE(serialTask);
        // CHECK_HEAP_AVAILABLE();
        // serial_bluetooth_tx_byte(0x01);
        uint8_t tmp = 0x01;
        HAL_UART_Transmit(&huart2, &tmp, 1, 100);
        tick += delayTick;
        osDelayUntil(tick);
    }
}
