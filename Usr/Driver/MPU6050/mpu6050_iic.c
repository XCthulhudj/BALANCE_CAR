#include "mpu6050_iic.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "bsp_i2c.h"
#include "mpu6050_reg.h"
#include "usr_delay.h"

#define MPU6050_ADDRESS 0xD0

static mpu6050_data_t mpu6050_data;

static void mpu6050_parser(void *arg);

mpu6050_data_t* mpu6050_init(void){
	static uint8_t data[6][6] = {{MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_2, MPU6050_SMPLRT_DIV, MPU6050_CONFIG, MPU6050_GYRO_CONFIG, MPU6050_ACCEL_CONFIG}, 
		{0x01, 0x00, 0x09, 0x06, 0x18, 0x18}};
	uint8_t i;
	for(i = 0; i < 6; i++){
		i2c2_write_byte_memAddSize_8bit(data[0][i], data[1][i]);
	}
	i2c2_hook_register(mpu6050_parser, mpu6050_data.rawData, MPU6050_ADDRESS);
	return &mpu6050_data;
}

mpu6050_state_t mpu6050_update(void){
	i2c2_read_start_memAddSize_8bit(MPU6050_ACCEL_XOUT_H, 14);
	return MPU6050_OK;
}

static void mpu6050_parser(void *arg){
	(void)arg;
	mpu6050_data.acc_x = ((int16_t)mpu6050_data.rawData[0] << 8) | mpu6050_data.rawData[1];
	mpu6050_data.acc_y = ((int16_t)mpu6050_data.rawData[2] << 8) | mpu6050_data.rawData[3];
	mpu6050_data.acc_z = ((int16_t)mpu6050_data.rawData[4] << 8) | mpu6050_data.rawData[5];
	mpu6050_data.temperature = ((int16_t)mpu6050_data.rawData[6] << 8) | mpu6050_data.rawData[7];
	mpu6050_data.gyro_x = ((int16_t)mpu6050_data.rawData[8] << 8) | mpu6050_data.rawData[9];
	mpu6050_data.gyro_y = ((int16_t)mpu6050_data.rawData[10] << 8) | mpu6050_data.rawData[11];
	mpu6050_data.gyro_z = ((int16_t)mpu6050_data.rawData[12] << 8) | mpu6050_data.rawData[13];
}
