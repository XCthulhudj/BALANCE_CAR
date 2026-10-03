#include "mpu6050_iic.h"

#include <stdint.h>

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "stm32f1xx_hal.h"

#include "bsp_i2c.h"
#include "bsp_it.h"
#include "mpu6050_reg.h"
#include "usr_delay.h"
#include "usr_fusion.h"

#define MPU6050_ADDRESS 0xD0

static mpu6050_data_t mpu6050_data;

static void mpu6050_parser(void *arg);
static void mpu6050_read_start(void *arg);

mpu6050_data_t* mpu6050_init(void){
	i2c2_hook_register(mpu6050_parser, mpu6050_data.rawData, MPU6050_ADDRESS);

	i2c2_write_byte_memAddSize_8bit(MPU6050_PWR_MGMT_1, 0x80);
	osDelay(100);
	i2c2_write_byte_memAddSize_8bit(MPU6050_PWR_MGMT_1,0x01);
	osDelay(100);
	i2c2_write_byte_memAddSize_8bit(MPU6050_PWR_MGMT_2,0x00);
	i2c2_write_byte_memAddSize_8bit(MPU6050_SMPLRT_DIV,0x01);
	i2c2_write_byte_memAddSize_8bit(MPU6050_CONFIG,0x03);
	i2c2_write_byte_memAddSize_8bit(MPU6050_GYRO_CONFIG,0x18);
	i2c2_write_byte_memAddSize_8bit(MPU6050_ACCEL_CONFIG,0x08);
	i2c2_write_byte_memAddSize_8bit(MPU6050_INT_PIN_CFG, 0x00);
	i2c2_write_byte_memAddSize_8bit(MPU6050_INT_ENABLE, 0x01);

	exti_it_hook_register(mpu6050_read_start, 5);

	mpu6050_data.fusion_data.q[0] = 1;
	mpu6050_data.cali_sig = 1;
	return &mpu6050_data;
}

mpu6050_state_t mpu6050_update(void){
	if(mpu6050_data.cali_sig == 1){
		if(mpu6050_data.cali.cnt >= 499){
            mpu6050_data.cali.gyro_x = (float)mpu6050_data.cali.sum[0] / mpu6050_data.cali.cnt;
            mpu6050_data.cali.gyro_y = (float)mpu6050_data.cali.sum[1] / mpu6050_data.cali.cnt;
            mpu6050_data.cali.gyro_z = (float)mpu6050_data.cali.sum[2] / mpu6050_data.cali.cnt;

            mpu6050_data.cali.sum[0] = 0;
            mpu6050_data.cali.sum[1] = 0;
            mpu6050_data.cali.sum[2] = 0;
            mpu6050_data.cali.cnt = 0;

            mpu6050_data.cali_sig = 0;
        }
	}
	mpu6050_data.fusion_data.raw.ax = mpu6050_data.raw.acc_x;
	mpu6050_data.fusion_data.raw.ay = mpu6050_data.raw.acc_y;
	mpu6050_data.fusion_data.raw.az = mpu6050_data.raw.acc_z;
	mpu6050_data.fusion_data.raw.gx = mpu6050_data.raw.gyro_x - mpu6050_data.cali.gyro_x;
	mpu6050_data.fusion_data.raw.gy = mpu6050_data.raw.gyro_y - mpu6050_data.cali.gyro_y;
	mpu6050_data.fusion_data.raw.gz = mpu6050_data.raw.gyro_z - mpu6050_data.cali.gyro_z;
	mpu6050_data.temperature = mpu6050_data.raw.temperature / 340.0f + 36.53f;
	if(mpu6050_data.cali_sig == 0){
		usr_fusion_imu_update(&mpu6050_data.fusion_data);
	}

	if(mpu6050_data.rx_sig == 1){
		mpu6050_data.rx_sig = 0;
		i2c2_read_start_memAddSize_8bit(MPU6050_ACCEL_XOUT_H, 14);
	}

	return MPU6050_OK;
}

static void mpu6050_parser(void *arg){
	(void)arg;
	mpu6050_data.raw.acc_x = ((int16_t)mpu6050_data.rawData[0] << 8) | mpu6050_data.rawData[1];
	mpu6050_data.raw.acc_y = ((int16_t)mpu6050_data.rawData[2] << 8) | mpu6050_data.rawData[3];
	mpu6050_data.raw.acc_z = ((int16_t)mpu6050_data.rawData[4] << 8) | mpu6050_data.rawData[5];
	mpu6050_data.raw.temperature = ((int16_t)mpu6050_data.rawData[6] << 8) | mpu6050_data.rawData[7];
	mpu6050_data.raw.gyro_x = ((int16_t)mpu6050_data.rawData[8] << 8) | mpu6050_data.rawData[9];
	mpu6050_data.raw.gyro_y = ((int16_t)mpu6050_data.rawData[10] << 8) | mpu6050_data.rawData[11];
	mpu6050_data.raw.gyro_z = ((int16_t)mpu6050_data.rawData[12] << 8) | mpu6050_data.rawData[13];

	if(mpu6050_data.cali_sig == 1){
        mpu6050_data.cali.sum[0] += mpu6050_data.raw.gyro_x;
        mpu6050_data.cali.sum[1] += mpu6050_data.raw.gyro_y;
        mpu6050_data.cali.sum[2] += mpu6050_data.raw.gyro_z;
        mpu6050_data.cali.cnt++;
	}
}

static void mpu6050_read_start(void *arg){
	(void)arg;
	mpu6050_data.rx_sig = 1;
}
