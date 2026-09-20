#include "bsp_it.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"
#include "oled_iic_4.h"
#include "bsp_uart.h"

extern DMA_HandleTypeDef hdma_i2c1_tx;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        uart1_send();
    }else if(huart->Instance == USART2){
        uart2_send();
    }else{

    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){
        oled_send();
    }else if(hi2c->Instance ==I2C2){
        
    }else{

    }
}


