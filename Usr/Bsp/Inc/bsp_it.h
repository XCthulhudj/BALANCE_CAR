#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*exti_event_hook_t)(void *arg);

typedef struct{
    exti_event_hook_t exti1_event;
    exti_event_hook_t exti5_event;
}exti_it_t;

void exti_it_hook_register(exti_event_hook_t func, uint8_t channel);

#ifdef __cplusplus
}
#endif
