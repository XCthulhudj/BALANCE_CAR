#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usr_queue.h"

#define I2C1_QUEUE_SIZE 256
#define I2C1_ADDRESS 0x78

typedef enum{
    I2C_SUCCESS = 0,
    I2C_BUSY,
    I2C_IDLE,
    I2C_WARNING_OVERLOAD
}i2c_state_t;

typedef struct{
    usr_queue_t queue;
    volatile i2c_state_t state;
    volatile uint16_t tx_width;
}i2c_tx_t;

i2c_state_t i2c1_check(void);
i2c_state_t i2c1_write_byte(uint8_t data);
i2c_state_t i2c1_write_data(const void *data_ptr, uint16_t size);

#ifdef __cplusplus
}
#endif
