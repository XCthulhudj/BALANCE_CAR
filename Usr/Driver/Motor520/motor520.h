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

typedef struct{
    volatile int32_t cn1_enc;
    volatile int32_t cn2_enc;
    int32_t cn1_set;
    int32_t cn2_set;
}motor520_t;

motor520_t* motor520_init(void);
motor520_state_t motor520_update(void);
motor520_state_t motor520_rpm_load(void);

#ifdef __cplusplus
}
#endif
