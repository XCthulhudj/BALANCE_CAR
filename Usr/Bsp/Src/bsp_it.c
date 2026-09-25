#include "bsp_it.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        uart1_check();
    }else if(huart->Instance == USART2){
        uart2_check();
    }else{

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        //never get here
    }else if(huart->Instance == USART2){

    }else{

    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){
        i2c1_check();
    }else if(hi2c->Instance ==I2C2){
        
    }else{

    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){

    }else if(hi2c->Instance ==I2C2){
        
    }else{

    }
}


