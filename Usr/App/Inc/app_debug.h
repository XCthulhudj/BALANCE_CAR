#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "app_config.h"
#include "usr_delay.h"

#define VOFA_FRAME_TAIL 0x7f800000UL

#ifdef DEBUG

#define DEBUG_PRINT(fmt, ...) \
do{printf("[DEBUG] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__);}while(0)

#define VOFA_SEND_FLOATS(...) do { \
    float _vofa_buf[] = { __VA_ARGS__ }; \
    static const uint32_t _vofa_tail = VOFA_FRAME_TAIL; \
    serial_debug_tx_ptr(_vofa_buf, sizeof(_vofa_buf)); \
    serial_debug_tx_ptr((void*)&_vofa_tail, sizeof(_vofa_tail)); \
} while(0)

#define CHECK_STACK_AVAILABLE(c) do { \
    UBaseType_t wm = uxTaskGetStackHighWaterMark(NULL); \
    if (usr_delay_ms(500) == USR_DELAY_SUCCESS_END) \
        printf("%s free stack: %lu words (%lu bytes)\n", \
               #c, \
               (unsigned long)wm, \
               (unsigned long)wm * sizeof(StackType_t)); \
} while(0)

#define CHECK_HEAP_AVAILABLE() do{ \
    if (usr_delay_ms(500) == USR_DELAY_SUCCESS_END){ \
        size_t min_free = xPortGetMinimumEverFreeHeapSize(); \
        printf("min ever free heap: %u bytes\n", (unsigned int)min_free); \
    } \
}while(0)

#elifndef DEBUG

#define DEBUG_PRINT(fmt, ...) ((void)0)
#define VOFA_PRINT(x) ((void)0)
#define CHCHECK_STACK_AVAILABLE(c) ((void)0)
#define CHECK_HEAP_AVAILABLE() ((void)0)

#endif

#ifdef __cplusplus
}
#endif
