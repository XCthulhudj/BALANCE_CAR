#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "struct_typedef.h"

#include "usr_pid.h"
#include "mpu6050_iic.h"
#include "rc.h"
#include "hc_sr04.h"
#include "motor520.h"
#include "app_debug.h"

typedef enum{
    MOTION_OK = 0,
    MOTION_ERROR,
    MOTION_BUSY,
    MOTION_TIMEOUT,
    MOTION_WARNING
}motion_state_t;

typedef struct{
    struct{
        pid_t pitch;
        pid_t vel;
        pid_t turn;
    }pid;

    struct{
        float q[4];
        float euler[3]; //roll pitch yaw
        float acc[3];   //x y z
        float gyro[3];  //x y z

        float v_enc_set;
        float wz_enc_set;

        int32_t v_enc;
    }object;

    mpu6050_data_t *imu_ptr;
    rc_t *rc_ptr;
    hc_sr04_t *hc_sr04_ptr;
    motor520_t *motor520_ptr;
    app_debug_t *app_debug_ptr;
}motion_t;

#ifdef __cplusplus
}
#endif
