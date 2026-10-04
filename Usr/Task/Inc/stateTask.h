#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct{
    uint8_t flag_cali_end : 1;
    uint8_t flag_state : 7;
}state_t;

#ifdef __cplusplus
}
#endif
