#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

#define TASK_FREQ_SERIAL 100u
#define TASK_FREQ_MOTIONCTRL 500u
#define TASK_FREQ_STATE 50u

#define TASK_INIT_DELAY_SERIAL 500u
#define TASK_INIT_DELAY_MOTIONCTRL 1000u
#define TASK_INIT_DELAY_STATE 500u

typedef struct{
    /*线程任务*/
    struct{
        osThreadId_t serial;
        osThreadId_t motionCtrl;
        osThreadId_t state;
    }thread;
    
    /*消息队列*/
    struct{
        osMessageQueueId_t serial;
        osMessageQueueId_t motionCtrl;
        osMessageQueueId_t state;
    }msgq;
    /*事件组*/
    struct{
        osEventFlagsId_t eventReceive;
    }events;

#ifdef DEBUG
    struct{
        UBaseType_t serial;
        UBaseType_t motionCtrl;
        UBaseType_t state;
    }stack_water_mark; /* stack使用 */

    struct{
        float serial;
        float motionCtrl;
        float state;
    }freq; /* 任务运行频率 */

    struct{
        float serial;
        float motionCtrl;
        float state;
    }last_up_time; /* 任务最近运行时 */
#endif

}task_t;

extern task_t task_struct;
extern const osThreadAttr_t attr_serial;
extern const osThreadAttr_t attr_motionCtrl;
extern const osThreadAttr_t attr_state;
extern const osEventFlagsAttr_t attr_event;

void serialTask(void *argument);
void motionCtrlTask(void *argument);
void stateTask(void *argument);

#ifdef __cplusplus
}
#endif
