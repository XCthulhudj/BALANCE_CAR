#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

typedef struct{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t temperature;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    uint8_t rawData[14];
}mpu6050_data_t;

mpu6050_data_t* mpu6050_init(void);
void mpu6050_update(void);

#ifdef __cplusplus
}
#endif
