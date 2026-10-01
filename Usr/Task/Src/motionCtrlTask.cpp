#include "motionCtrlTask.h"

#include <stdio.h>

#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "stm32f1xx_hal.h"

#include "usr_config.h"
#include "app_debug.h"
#include "mpu6050_iic.h"
#include "usr_delay.h"
#include "struct_typedef.h"
#include "usr_pid.h"
#include "hc_sr04.h"
#include "rc.h"

extern IWDG_HandleTypeDef hiwdg;

#define PID_KP 1.0f
#define PID_KI 0.1f
#define PID_KD 0.01f

motion_t motion;

static motion_state_t motion_init(void);
static motion_state_t motion_update(void);

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

        if(usr_delay_ms(100) == USR_DELAY_SUCCESS_END){
            VOFA_SEND_FLOATS(motion.imu_ptr->fusion_data.acc_x, 
                motion.imu_ptr->fusion_data.acc_y, 
                motion.imu_ptr->fusion_data.acc_z, 
                motion.imu_ptr->fusion_data.gyro_x, 
                motion.imu_ptr->fusion_data.gyro_y, 
                motion.imu_ptr->fusion_data.gyro_z, 
                motion.imu_ptr->fusion_data.roll, 
                motion.imu_ptr->fusion_data.pitch, 
                motion.imu_ptr->fusion_data.yaw, 
                motion.imu_ptr->temperature,
                motion.hc_sr04_ptr->distance
            );
        }

        tick += delayTick;
        osDelayUntil(tick);
    }
}

static motion_state_t motion_init(void){
    fp32 Kpid[3] = {PID_KP, PID_KI, PID_KD};
    usr_pid_init(&motion.pid, PID_POSITION, Kpid, 1.0f, 1.0f);

    motion.imu_ptr = mpu6050_init();

    motion.rc_ptr = rc_init();

    motion.hc_sr04_ptr = hc_sr04_init();

    return MOTION_OK;
}

static motion_state_t motion_update(void){
    mpu6050_update();
    motion.hc_sr04_ptr->trigger_func();
    motion.hc_sr04_ptr->echo_func();
    return MOTION_OK;
}
