#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

typedef enum{
    HC_SR04_OK = 0,
    HC_SR04_ERROR,
    HC_SR04_BUSY,
    HC_SR04_TIMEOUT,
    HC_SR04_INVALID
}hc_sr04_state_t;

typedef struct{
    struct{
        volatile uint8_t done_sig;
        volatile uint32_t start_timeStamp;
        volatile uint32_t period;
    }echo;

    float distance; //cm

    hc_sr04_state_t (*trigger_func)(void);
    hc_sr04_state_t (*echo_func)(void);
}hc_sr04_t;

hc_sr04_t* hc_sr04_init(void);

#ifdef __cplusplus
}
#endif
