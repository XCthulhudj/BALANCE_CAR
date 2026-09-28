#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usr_queue.h"

typedef enum{
    UART_OK = 0,
    UART_ERROR,
    UART_BUSY,
    UART_TIMEOUT,
    UART_IDLE,
    UART_WARNING_OVERLOAD,
}uart_state_t;

typedef void (*uart_event_hook_t)(void *arg);

typedef struct {
    usr_queue_t queue;
    volatile uart_state_t state;
    volatile uint16_t tx_width;
}uart_tx_t;

typedef struct{
    uart_event_hook_t irq_parser;
    uint8_t *buff_ptr;
    uint16_t size;
}uart_rx_t;

uart_state_t uart1_write_byte(uint8_t data);
uart_state_t uart1_write_data(const void *data_ptr, uint16_t size);
uart_state_t uart1_tx_handler(void);

uart_state_t uart1_read_start(void);
uart_state_t uart1_rx_hook_register(uart_event_hook_t func, uint8_t *buff_ptr, uint16_t size);
uart_state_t uart1_rx_handler(void);

uart_state_t uart2_write_byte(uint8_t data);
uart_state_t uart2_write_data(const void *data_ptr, uint16_t size);
uart_state_t uart2_tx_handler(void);

uart_state_t uart2_read_start(void);
uart_state_t uart2_rx_hook_register(uart_event_hook_t func, uint8_t *buff_ptr, uint16_t size);
uart_state_t uart2_rx_handler(void);

#ifdef __cplusplus
}
#endif
