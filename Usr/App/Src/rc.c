#include "rc.h"

#include <stdint.h>

#include "app_config.h"

static rc_t rc;

static void rc_parser(void *arg);

rc_t* rc_init(void){
    serial_bluetooth_rx_hook_register(rc_parser, rc.rx_buff);
    return &rc;
}

static void rc_parser(void *arg){
    (void)arg;
    if( rc.rx_buff[0] == RC_DIR_AHEAD  || 
        rc.rx_buff[0] == RC_DIR_BACK   || 
        rc.rx_buff[0] == RC_DIR_LEFT   || 
        rc.rx_buff[0] == RC_DIR_RIGHT  || 
        rc.rx_buff[0] == RC_DIR_BRAKE){
            rc.dir = rc.rx_buff[0];
        }
    serial_bluetooth_read_start();
}
