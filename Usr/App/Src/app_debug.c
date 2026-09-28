#include "app_debug.h"

#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>

#include "app_config.h"

#define APP_DEBUG_RX_BUFF_SIZE 1

static uint8_t app_debug_rx_buff[APP_DEBUG_RX_BUFF_SIZE];

static void app_debug_parser(void *arg);

void app_debug_init(void){
#ifdef DEBUG
    serial_debug_rx_hook_register(app_debug_parser, app_debug_rx_buff);
#endif
}

static void app_debug_parser(void *arg){
    (void)arg;
    serial_debug_tx_data(app_debug_rx_buff);
    serial_debug_read_start();
}

int _write(int file, char *ptr, int len) {
    (void)file;
    serial_debug_tx_ptr(ptr, len);
    return len;
}
