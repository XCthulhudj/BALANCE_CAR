#pragma  once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdio.h>

#include "bsp_uart.h"

#define serial_debug_init() uart1_init()
#define serial_debug_tx_byte(x) uart1_write_byte(x)
#define serial_debug_tx_data(x) uart1_write_data(&(x), sizeof(x))
#define serial_debug_tx_ptr(p, n) uart1_write_data(p, n)
#define serial_bluetooth_init() uart2_init()
#define serial_bluetooth_tx_byte(x) uart2_write_byte(x)
#define serial_bluetooth_tx_data(x) uart2_write_data(&(x), sizeof(x))
#define serial_bluetooth_tx_ptr(p, n) uart2_write_data(p, n)


#ifdef __cplusplus
}
#endif
