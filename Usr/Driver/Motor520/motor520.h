#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

typedef enum{
    MOTOR520_OK = 0,
    MOTOR520_ERROR,
    MOTOR520_BUSY,
    MOTOR520_TIMEOUT
}motor520_state_t;

#ifdef __cplusplus
}
#endif
