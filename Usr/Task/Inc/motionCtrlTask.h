#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "struct_typedef.h"

#include "mpu6050_iic.h"

typedef enum{
    MOTION_OK = 0,
    MOTION_ERROR,
    MOTION_WARNING,
}motion_state_t;

typedef struct{
    mpu6050_data_t *raw;
    fp32 acc_x;
    fp32 acc_y;
    fp32 acc_z;
    fp32 gyro_x;
    fp32 gyro_y;
    fp32 gyro_z;
    fp32 temperature;
}imu_t;

typedef struct{
    imu_t imu;
}motion_t;

#ifdef __cplusplus
}
#endif
