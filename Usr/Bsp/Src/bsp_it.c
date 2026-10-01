#include "bsp_it.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "bsp_uart.h"
#include "bsp_i2c.h"

exti_it_t exti_it = {
    .exti1_event = NULL,
    .exti5_event = NULL
};

static void exti_it_1_handler(void);
static void exti_it_5_handler(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        uart1_tx_handler();
    }else if(huart->Instance == USART2){
        uart2_tx_handler();
    }else{

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        uart1_rx_handler();
    }else if(huart->Instance == USART2){
        uart2_rx_handler();
    }else{

    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){
        i2c1_tx_handler();
    }else if(hi2c->Instance ==I2C2){

    }else{

    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C2){
        i2c2_rx_parser_handler();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    switch(GPIO_Pin){
        case GPIO_PIN_1:{
            exti_it_1_handler();
        }break;
        case GPIO_PIN_5:{
            exti_it_5_handler();
        }break;
        default: break;
    }
}

void exti_it_hook_register(exti_event_hook_t func, uint8_t channel){
    switch(channel){
        case 1:{
            exti_it.exti1_event = func;
        }break;
        case 5:{
            exti_it.exti5_event = func;
        }break;
        default: break;
    }
}

static void exti_it_1_handler(void){
    if(exti_it.exti1_event == NULL) return;
    else exti_it.exti1_event(NULL);
}

static void exti_it_5_handler(void){
    if(exti_it.exti5_event == NULL) return;
    else exti_it.exti5_event(NULL);
}

