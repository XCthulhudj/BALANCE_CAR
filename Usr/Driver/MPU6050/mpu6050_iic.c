#include "mpu6050_iic.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "bsp_i2c.h"
#include "mpu6050_reg.h"
#include "usr_delay.h"

#define MPU6050_ADDRESS 0xD0

extern I2C_HandleTypeDef hi2c2;

mpu6050_data_t mpu6050_data;

mpu6050_data_t* mpu6050_init(void){
	static uint8_t data[] = {0x01, 0x00, 0x09, 0x06, 0x18, 0x18};
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data[0], 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &data[1], 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &data[2], 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data[3], 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data[4], 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data[5], 1, 100);
	return &mpu6050_data;
}

void mpu6050_update(void){
	HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, mpu6050_data.rawData,14);
}


