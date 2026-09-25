#include "app_debug.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/stat.h>

#include "app_config.h"

int _write(int file, char *ptr, int len) {
    (void)file;
    serial_debug_tx_ptr(ptr, len);
    return len;
}
