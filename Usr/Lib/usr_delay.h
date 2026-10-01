#pragma  once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f1xx_hal.h"

#ifndef SYSTEM_CLOCK
#define SYSTEM_CLOCK 72000000UL
#endif

typedef enum{
    USR_DELAY_SUCCESS_END = 0,
    USR_DELAY_SUCCESS_WAITING,
    USR_DELAY_ERROR_RUNTIME
}usr_delay_t;

#define usr_delay_ms(ms)                                    \
({                                                          \
    static uint32_t sysTick = 0;                            \
    usr_delay_t ret;                                        \
                                                            \
    if (uwTick - sysTick < (ms))                            \
    {                                                       \
        ret = USR_DELAY_SUCCESS_WAITING;                    \
    }                                                       \
    else                                                    \
    {                                                       \
        sysTick = uwTick;                                   \
        ret = USR_DELAY_SUCCESS_END;                        \
    }                                                       \
                                                            \
    ret;                                                    \
})

#define usr_delay_us(us)                                            \
({                                                                  \
    static uint32_t sysTick = 0;                                    \
    usr_delay_t ret;                                                \
                                                                    \
    if ((DWT->CYCCNT - sysTick) < ((us) * (SYSTEM_CLOCK / 1000000)))\
    {                                                               \
        ret = USR_DELAY_SUCCESS_WAITING;                            \
    }                                                               \
    else                                                            \
    {                                                               \
        sysTick = DWT->CYCCNT;                                      \
        ret = USR_DELAY_SUCCESS_END;                                \
    }                                                               \
                                                                    \
    ret;                                                            \
})

#define DWT_INIT() do{ \
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
    DWT->CYCCNT = 0; \
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
}while(0)

static inline void delay_us_blocking(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYSTEM_CLOCK / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

#ifdef __cplusplus
}
#endif
