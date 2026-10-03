#include "app_debug.h"

#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>

#include "app_config.h"

app_debug_t app_debug;

static void app_debug_parser(void *arg);

app_debug_t* app_debug_init(void){
#ifdef DEBUG
    serial_debug_rx_hook_register(app_debug_parser, app_debug.rx_buff);
#endif
return &app_debug;
}

static void app_debug_parser(void *arg){
    (void)arg;
    /* Test module */
    if(app_debug.rx_buff[0] == 'a'){
        app_debug.flag = 1;
    }else if(app_debug.rx_buff[0] == 'z'){
        app_debug.flag = 0;
    }
    serial_debug_tx_data(app_debug.rx_buff);
    serial_debug_read_start();
}

int _write(int file, char *ptr, int len) {
    (void)file;
    serial_debug_tx_ptr(ptr, len);
    return len;
}
