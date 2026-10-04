#include "stateTask.h"

#include <stdint.h>
#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "main.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"
#include "ws2812_driver.h"
#include "oled_iic_4.h"

state_t state;

static void state_check(void);
static void state_set(void);
static void usr_led_blink(uint32_t time_gap);

void stateTask(void *argument){
    (void)argument;

    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_STATE;
    
    osDelay(TASK_INIT_DELAY_STATE);
    uint32_t tick = osKernelGetTickCount();

    while(1){
        CHECK_STACK_AVAILABLE(stateTask);

        state_check();

        state_set();

        tick += delayTick;
        osDelayUntil(tick);
    }
}

static void state_check(void){
    int32_t flag_wait = osThreadFlagsWait(CALI_FLAG, osFlagsWaitAny, 0);
    if(flag_wait >= 0 && (flag_wait & CALI_FLAG)){
        state.flag_cali_end = 1;
    }
}

static void state_set(void){
    if(!state.flag_cali_end){
        usr_led_blink(100);
    }else{
        usr_led_blink(1000);
    }
}

static void usr_led_blink(uint32_t time_gap){
    if(usr_delay_ms(time_gap) == USR_DELAY_SUCCESS_END){
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}
