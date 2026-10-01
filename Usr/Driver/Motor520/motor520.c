#include "motor520.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

void motor520_init(void){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //PWMB
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //PWMA
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //AIN1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //AIN2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); //BIN1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //BIN2
}
