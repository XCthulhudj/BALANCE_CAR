#include "initTask.h"

#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f1xx_hal.h"
#include "iwdg.h"

#include "usr_config.h"
#include "struct_typedef.h"
#include "ws2812_driver.h"
#include "usr_delay.h"

void initTask(void *argument){
  (void)argument;

  osKernelLock();

  task_struct.thread.serial = osThreadNew(serialTask, NULL, &attr_serial);
  task_struct.thread.motionCtrl = osThreadNew(motionCtrlTask, NULL, &attr_motionCtrl);
  task_struct.thread.state = osThreadNew(stateTask, NULL, &attr_state);

  task_struct.msgq.serial = osMessageQueueNew(16u, sizeof(uint32_t), NULL);
  task_struct.msgq.motionCtrl = osMessageQueueNew(16u, sizeof(uint32_t), NULL);
  task_struct.msgq.state = osMessageQueueNew(16u, sizeof(uint32_t), NULL);
  
  task_struct.events.eventReceive = osEventFlagsNew(&attr_event);

  osKernelUnlock();
  osThreadTerminate(osThreadGetId());
  // while(1){
  //   HAL_IWDG_Refresh(&hiwdg);

  //   osDelay(20);
  // }
}
