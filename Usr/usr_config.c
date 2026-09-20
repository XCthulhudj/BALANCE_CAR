#include "usr_config.h"

task_t task_struct;

const osThreadAttr_t attr_serial = {
    .name = "serial",
    .priority = osPriorityRealtime,
    .stack_size = 128 * 4,
};

const osThreadAttr_t attr_motionCtrl = {
    .name = "motionCtrl",
    .priority = osPriorityRealtime,
    .stack_size = 512 * 4,
};

const osThreadAttr_t attr_state = {
    .name = "state",
    .priority = osPriorityRealtime,
    .stack_size = 128 * 4,
};

const osEventFlagsAttr_t attr_event = {
    .name = "event",
};
