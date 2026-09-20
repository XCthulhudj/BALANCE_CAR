#include "usr_queue.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

void usr_queue_in(usr_queue_t *queue_ptr, const uint8_t *src){
    uint32_t index = (queue_ptr->tail + 1) % queue_ptr->size;

    if(index == queue_ptr->head) return;

    uint8_t *dest = queue_ptr->queue_buff_ptr + queue_ptr->tail * sizeof(uint8_t);

    memcpy(dest, src, sizeof(uint8_t));

    queue_ptr->tail = index;
}

void usr_queue_out(usr_queue_t *queue_ptr, uint8_t *dest){
    if(queue_ptr->head == queue_ptr->tail) return;

    uint8_t *src = queue_ptr->queue_buff_ptr + queue_ptr->head * sizeof(uint8_t);

    memcpy(dest, src, sizeof(uint8_t));

    queue_ptr->head = (queue_ptr->head + 1) % queue_ptr->size;
}

void usr_queue_in_array(usr_queue_t *queue_ptr, const uint8_t *arr_ptr, uint32_t size){
    uint32_t i = 0;
    for(; i < size; i++){
        usr_queue_in(queue_ptr, &arr_ptr[i]);
    }
}
