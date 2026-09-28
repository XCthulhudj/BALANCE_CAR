#pragma  once

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdio.h>

#include "bsp_uart.h"

#define serial_debug_tx_byte(x) uart1_write_byte(x)
#define serial_debug_tx_data(buff) uart1_write_data(&(buff), sizeof(buff))
#define serial_debug_tx_ptr(p, n) uart1_write_data(p, n)

#define serial_debug_rx_hook_register(func, buff) uart1_rx_hook_register(func, buff, sizeof(buff))
#define serial_debug_read_start() uart1_read_start()

#define serial_bluetooth_tx_byte(x) uart2_write_byte(x)
#define serial_bluetooth_tx_data(buff) uart2_write_data(&(buff), sizeof(buff))
#define serial_bluetooth_tx_ptr(p, n) uart2_write_data(p, n)

#define serial_bluetooth_rx_hook_register(func, buff) uart2_rx_hook_register(func, buff, sizeof(buff))
#define serial_bluetooth_read_start() uart2_read_start()

#ifdef __cplusplus
}
#endif
