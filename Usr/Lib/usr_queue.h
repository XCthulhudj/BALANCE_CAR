#pragma once

#include <stdint.h>

typedef struct{
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint8_t *queue_buff_ptr;
}usr_queue_t;

void usr_queue_in(usr_queue_t *queue_ptr, const uint8_t *src);
void usr_queue_out(usr_queue_t *queue_ptr, uint8_t *dest);
void usr_queue_in_array(usr_queue_t *queue_ptr, const uint8_t *arr_ptr, uint32_t size);
