#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usr_queue.h"

#define I2C1_ADDRESS 0x78
#define I2C1_TX_QUEUE_SIZE 256

typedef enum{
    I2C_OK = 0,
    I2C_ERROR,
    I2C_BUSY,
    I2C_TIMEOUT,
    I2C_IDLE,
    I2C_WARNING_OVERLOAD,
    I2C_WARNING_EMPTY,
    I2C_WARNING_NULL
}i2c_state_t;

typedef void (*i2c_event_hook_t)(void *arg);

typedef struct{
    usr_queue_t queue;
    volatile i2c_state_t state;
    volatile uint16_t tx_width;
}i2c_tx_t;

typedef struct{
    i2c_event_hook_t irq_parser;
    uint8_t *buff_ptr;
    uint16_t dev_id;
}bsp_i2c_t;

i2c_state_t i2c1_write_byte(uint8_t data);
i2c_state_t i2c1_write_data(const void *data_ptr, uint16_t size);
i2c_state_t i2c1_tx_handler(void);

i2c_state_t i2c2_write_byte_memAddSize_8bit(uint16_t mem_address, uint8_t data);
i2c_state_t i2c2_write_data_memAddSize_8bit(uint16_t mem_address, uint16_t size, uint8_t *data_ptr);
i2c_state_t i2c2_read_start_memAddSize_8bit(uint16_t mem_address, uint16_t size);
i2c_state_t i2c2_read_byte_memAddSize_8bit_blocking(uint16_t devAddr, uint8_t memAddr, uint8_t *data);
i2c_state_t i2c2_hook_register(i2c_event_hook_t func_parser, uint8_t *buff_ptr, uint8_t dev_id);
i2c_state_t i2c2_rx_parser_handler(void);

#ifdef __cplusplus
}
#endif
