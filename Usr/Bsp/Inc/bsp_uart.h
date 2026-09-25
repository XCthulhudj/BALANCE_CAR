#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usr_queue.h"

typedef enum{
    UART_SUCCESS = 0,
    UART_BUSY,
    UART_IDLE,
    UART_WARNING_OVERLOAD,
}uart_state_t;

typedef struct {
    usr_queue_t queue;
    volatile uart_state_t state;
    volatile uint16_t tx_width;
}uart_tx_t;

typedef struct{
    void (*parser)(uint8_t *src);
    uint8_t *rx_buf;
    uint16_t rx_len;
    uint16_t rx_buf_size;
    uint8_t  frame_ready;
}uart_rx_t;

uart_state_t uart1_init(void);
uart_state_t uart1_check(void);
uart_state_t uart1_write_byte(uint8_t data);
uart_state_t uart1_write_data(const void *data_ptr, uint16_t size);
uart_state_t uart2_init(void);
uart_state_t uart2_check(void);
uart_state_t uart2_write_byte(uint8_t data);
uart_state_t uart2_write_data(const void *data_ptr, uint16_t size);

#ifdef __cplusplus
}
#endif
