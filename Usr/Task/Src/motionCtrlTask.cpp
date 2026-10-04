#include "motionCtrlTask.h"

#include <stdint.h>
#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "main.h"

#include "usr_config.h"
#include "app_debug.h"
#include "usr_delay.h"
#include "usr_pid.h"
#include "usr_fusion.h"
#include "usr_algorithm.h"
#include "struct_typedef.h"
#include "mpu6050_iic.h"
#include "motor520.h"
#include "hc_sr04.h"
#include "rc.h"

extern IWDG_HandleTypeDef hiwdg;

static fp32 pid_pitch[3] = {400.0f, 0.0f, 1.0f};
static fp32 pid_vel[3] = {0.1f, 0.05f, 0.0f};
static fp32 pid_turn[3] = {10.0f, 0.0f, 0.6f};

motion_t motion;

static motion_state_t motion_init(void);
static motion_state_t motion_update(void);
static motion_state_t motion_set(void);
static motion_state_t motion_check(void);
static motion_state_t motion_load(void);

void motionCtrlTask(void *argument){
    (void)argument;
    const uint32_t delayTick = osKernelGetTickFreq() / TASK_FREQ_MOTIONCTRL;
    motion_init();

    osDelay(TASK_INIT_DELAY_MOTIONCTRL);
    uint32_t tick = osKernelGetTickCount();
    while(1){
        HAL_IWDG_Refresh(&hiwdg);
        // CHECK_STACK_AVAILABLE(motionCtrlTask);
        motion_update();

        motion_set();

        motion_check();

        motion_load();

        if(usr_delay_ms(100) == USR_DELAY_SUCCESS_END){
            VOFA_SEND_FLOATS(
                // motion.imu_ptr->fusion_data.acc_x, 
                // motion.imu_ptr->fusion_data.acc_y, 
                // motion.imu_ptr->fusion_data.acc_z, 
                // motion.imu_ptr->fusion_data.gyro_x, 
                // motion.imu_ptr->fusion_data.gyro_y, 
                // motion.imu_ptr->fusion_data.gyro_z, 
                motion.imu_ptr->fusion_data.roll, 
                motion.imu_ptr->fusion_data.pitch, 
                motion.imu_ptr->fusion_data.yaw, 
                // motion.imu_ptr->temperature,
                motion.hc_sr04_ptr->distance,
                motion.object.euler[0],
                motion.object.euler[1],
                motion.object.euler[2],
                // motion.object.acc[0],
                // motion.object.acc[1],
                // motion.object.acc[2],
                // motion.object.gyro[0],
                // motion.object.gyro[1],
                // motion.object.gyro[2],
                motion.object.v_enc_set,
                (fp32)motion.object.v_enc,
                (fp32)motion.app_debug_ptr->flag,
                motion.pid.vel.out,
                motion.pid.pitch.out,
                motion.pid.turn.out
            );
        }

        tick += delayTick;
        osDelayUntil(tick);
    }
}

static motion_state_t motion_init(void){
    usr_pid_init(&motion.pid.pitch, PID_POSITION, pid_pitch, 3000.0f, 100.0f);
    usr_pid_init(&motion.pid.vel, PID_POSITION, pid_vel, 100.0f, 100.0f);
    usr_pid_init(&motion.pid.turn, PID_POSITION, pid_turn, 100.0f, 100.0f);

    motion.imu_ptr = mpu6050_init();

    motion.rc_ptr = rc_init();

    motion.hc_sr04_ptr = hc_sr04_init();

    motion.motor520_ptr = motor520_init();

    motion.app_debug_ptr = app_debug_init();

    return MOTION_OK;
}

static motion_state_t motion_update(void){
    static fp32 yaw_offset = 0.0f;
    static int yaw_inited = 0;
    mpu6050_update();

    fp32 acc[3] = {motion.imu_ptr->fusion_data.acc_x, 
        motion.imu_ptr->fusion_data.acc_y, 
        motion.imu_ptr->fusion_data.acc_z};
    fp32 gyro[3] = {motion.imu_ptr->fusion_data.gyro_x, 
        motion.imu_ptr->fusion_data.gyro_y, 
        motion.imu_ptr->fusion_data.gyro_z};

    usr_coordinate_rotation_internal(motion.imu_ptr->fusion_data.q, motion.object.q, motion.object.euler, 0, 0, 1, 90);

    if (!yaw_inited){
        yaw_offset = -motion.object.euler[2];
        yaw_inited = 1;
    }
    motion.object.euler[2] += yaw_offset;

    usr_rq_vector(acc, motion.object.acc, 0, 0, 1, -90);
    usr_rq_vector(gyro, motion.object.gyro, 0, 0, 1, -90);

    motion.hc_sr04_ptr->trigger_func();
    motion.hc_sr04_ptr->echo_func();

    motor520_update();
    motion.object.v_enc = motion.motor520_ptr->cn1_enc - motion.motor520_ptr->cn2_enc;

    return MOTION_OK;
}

static motion_state_t motion_set(void){
    switch(motion.rc_ptr->dir){
        case RC_DIR_AHEAD:{
            motion.object.v_enc_set = 10;
            motion.object.wz_enc_set = 0;
        }break;
        case RC_DIR_BACK:{
            motion.object.v_enc_set = -10;
            motion.object.wz_enc_set = 0;
        }break;
        case RC_DIR_LEFT:{
            motion.object.v_enc_set = 0;
            motion.object.wz_enc_set = 5;
        }break;
        case RC_DIR_RIGHT:{
            motion.object.v_enc_set = 0;
            motion.object.wz_enc_set = -5;
        }break;
        case RC_DIR_BRAKE:{
            motion.object.v_enc_set = 0;
            motion.object.wz_enc_set = 0;
        }break;
        default:{
            motion.object.v_enc_set = 0;
            motion.object.wz_enc_set = 0;
        }break;
    }

    /* DEBUG */
    if(motion.app_debug_ptr->flag == 1){
        motion.object.v_enc_set = 10;
    }else{
        motion.object.v_enc_set = 0;
    }

    int32_t pwm_compare_set = 0, turn_compare_added = 0;
    usr_pid_caculate(&motion.pid.vel, motion.object.v_enc, motion.object.v_enc_set);
    pwm_compare_set = usr_pid_caculate(&motion.pid.pitch, motion.object.euler[1], motion.pid.vel.out);
    turn_compare_added = usr_pid_caculate(&motion.pid.turn, motion.object.gyro[2], motion.object.wz_enc_set);

    motion.motor520_ptr->cn1_set = -pwm_compare_set + turn_compare_added;
    motion.motor520_ptr->cn2_set = pwm_compare_set - turn_compare_added;
    return MOTION_OK;
}

static motion_state_t motion_check(void){
    static uint8_t once_flag[5];
    if(motion.imu_ptr->cali_sig == 1 && once_flag[0] == 0){
        once_flag[0] = 1;
        osThreadFlagsSet(task_struct.thread.state, CALI_FLAG);
    }
    return MOTION_OK;
}

static motion_state_t motion_load(void){
    motor520_rpm_load();
    return MOTION_OK;
}

