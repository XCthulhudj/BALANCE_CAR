#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum{
    RC_OK = 0,
    RC_ERROR,
    RC_BUSY,
    RC_TIMEOUT
}rc_state_t;

enum{
    RC_DIR_AHEAD = 0X01,
    RC_DIR_BACK = 0X02,
    RC_DIR_LEFT = 0X11,
    RC_DIR_RIGHT = 0X12,
    RC_DIR_BRAKE = 0XFF
};

typedef struct{
    uint8_t dir;
    uint8_t rx_buff[1];
}rc_t;

rc_t* rc_init(void);

#ifdef __cplusplus
}
#endif
