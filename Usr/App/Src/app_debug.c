#include "app_debug.h"

#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>

#include "app_config.h"

#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim1;

#define APP_DEBUG_RX_BUFF_SIZE 1

static uint8_t app_debug_rx_buff[APP_DEBUG_RX_BUFF_SIZE];

static void app_debug_parser(void *arg);

void app_debug_init(void){
#ifdef DEBUG
    serial_debug_rx_hook_register(app_debug_parser, app_debug_rx_buff);
#endif
}

static void app_debug_parser(void *arg){
    (void)arg;
    if(app_debug_rx_buff[0] == 'a'){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000);
    }else if(app_debug_rx_buff[0] == 'z'){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    serial_debug_tx_data(app_debug_rx_buff);
    serial_debug_read_start();
}

int _write(int file, char *ptr, int len) {
    (void)file;
    serial_debug_tx_ptr(ptr, len);
    return len;
}
