#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum{
    UART_SUCCESS = 0,
    UART_ERROR_OVERFLOW = -1,
}uart_state_t;

uart_state_t uart1_send(void);
uart_state_t uart1_write_byte(uint8_t data);
uart_state_t uart1_write_arr(const uint8_t *arr, uint16_t size);
uart_state_t uart2_send(void);
uart_state_t uart2_write_byte(uint8_t data);
uart_state_t uart2_write_arr(const uint8_t *arr, uint16_t size);


#ifdef __cplusplus
}
#endif
