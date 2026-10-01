#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

typedef struct{
    struct{
        int ax;
        int ay;
        int az;
        int gx;
        int gy;
        int gz;
    }raw;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float q[4];
    float roll;
    float pitch;
    float yaw;
}fusion_t;

void usr_fusion_imu_update(fusion_t *fusion_data);

#ifdef __cplusplus
}
#endif
