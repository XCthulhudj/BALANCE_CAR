#include "bsp_it.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "mpu6050_iic.h"

extern mpu6050_data_t mpu6050_data;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        uart1_tx_check();
    }else if(huart->Instance == USART2){
        uart2_tx_check();
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
        i2c1_tx_check();
    }else if(hi2c->Instance ==I2C2){

    }else{

    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){
        //never get here
    }else if(hi2c->Instance ==I2C2){
        
    }else{

    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        mpu6050_data.AccX =
            ((int16_t)mpu6050_data.rawData[0] << 8) |
             mpu6050_data.rawData[1];

        mpu6050_data.AccY =
            ((int16_t)mpu6050_data.rawData[2] << 8) |
             mpu6050_data.rawData[3];

        mpu6050_data.AccZ =
            ((int16_t)mpu6050_data.rawData[4] << 8) |
             mpu6050_data.rawData[5];

        mpu6050_data.temperature =
            ((int16_t)mpu6050_data.rawData[6] << 8) |
             mpu6050_data.rawData[7];

        mpu6050_data.GyroX =
            ((int16_t)mpu6050_data.rawData[8] << 8) |
             mpu6050_data.rawData[9];

        mpu6050_data.GyroY =
            ((int16_t)mpu6050_data.rawData[10] << 8) |
             mpu6050_data.rawData[11];

        mpu6050_data.GyroZ =
            ((int16_t)mpu6050_data.rawData[12] << 8) |
             mpu6050_data.rawData[13];
    }
}