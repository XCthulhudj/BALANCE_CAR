#pragma  once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f1xx_hal.h"

typedef enum{
    USR_DELAY_SUCCESS_END       = 1,
    USR_DELAY_SUCCESS_WAITING   = 0,
    USR_DELAY_ERROR_RUNTIME     = -1
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

#ifdef __cplusplus
}
#endif
