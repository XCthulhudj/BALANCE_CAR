#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "struct_typedef.h"

#include "mpu6050_iic.h"
#include "rc.h"
#include "usr_pid.h"
#include "hc_sr04.h"

typedef enum{
    MOTION_OK = 0,
    MOTION_ERROR,
    MOTION_WARNING,
}motion_state_t;

typedef struct{
    pid_t pid;
    mpu6050_data_t *imu_ptr;
    rc_t *rc_ptr;
    hc_sr04_t *hc_sr04_ptr;
}motion_t;

#ifdef __cplusplus
}
#endif
