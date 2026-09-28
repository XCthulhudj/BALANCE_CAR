#include "motionCtrlTask.h"

#include <stdio.h>

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "rc.h"
#include "task.h"

#include "stm32f1xx_hal.h"

#include "usr_config.h"
#include "app_debug.h"
#include "mpu6050_iic.h"
#include "usr_delay.h"
#include "struct_typedef.h"
#include "usr_pid.h"

extern IWDG_HandleTypeDef hiwdg;

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

        // if(usr_delay_ms(100) == USR_DELAY_SUCCESS_END){
        //     VOFA_SEND_FLOATS(motion.imu.acc_x, 
        //         motion.imu.acc_y, 
        //         motion.imu.acc_z, 
        //         motion.imu.gyro_x, 
        //         motion.imu.gyro_y, 
        //         motion.imu.gyro_z,
        //         motion.imu.temperature);
        // }

        tick += delayTick;
        osDelayUntil(tick);
    }
}

static motion_state_t motion_init(void){
    // fp32 Kpid[3] = {1.0f, 0.1f, 0.01f};
    motion.imu.raw = mpu6050_init();
    motion.rc = rc_init();
    // usr_pid_init(&motion.pid, PID_POSITION, Kpid, 1.0f, 1.0f);
    return MOTION_OK;
}

static motion_state_t motion_update(void){
    motion.imu.acc_x = (fp32)(motion.imu.raw->acc_x) / 2048.0f;
    motion.imu.acc_y = (fp32)(motion.imu.raw->acc_y) / 2048.0f;
    motion.imu.acc_z = (fp32)(motion.imu.raw->acc_z) / 2048.0f;
    motion.imu.gyro_x = (fp32)(motion.imu.raw->gyro_x) / 16.4f;
    motion.imu.gyro_y = (fp32)(motion.imu.raw->gyro_y)/ 16.4f;
    motion.imu.gyro_z = (fp32)(motion.imu.raw->gyro_z)/ 16.4f;
    motion.imu.temperature = (fp32)(motion.imu.raw->temperature) / 340.0f + 36.53f;
    mpu6050_update();
    return MOTION_OK;
}
