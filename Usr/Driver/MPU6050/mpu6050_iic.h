#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

#include "usr_fusion.h"

#define MPU6050_DATA_BUFF 14

typedef enum{
    MPU6050_OK = 0
}mpu6050_state_t;

typedef struct{
    struct{
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t temperature;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    }raw;
    float temperature;
    uint8_t rawData[MPU6050_DATA_BUFF];
    fusion_t fusion_data;
    volatile uint8_t rx_sig;
}mpu6050_data_t;

mpu6050_data_t* mpu6050_init(void);
mpu6050_state_t mpu6050_update(void);

#ifdef __cplusplus
}
#endif
