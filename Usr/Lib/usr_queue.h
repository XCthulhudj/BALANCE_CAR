#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum{
    QUEUE_OK = 0,
    QUEUE_ERROR,
    QUEUE_EMPTY,
    QUEUE_OVERLOAD
}usr_queue_state_t;

typedef struct{
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint32_t usedSize;
    uint8_t *queue_buff_ptr;
}usr_queue_t;

usr_queue_state_t usr_queue_in(usr_queue_t *queue_ptr, const uint8_t *src);
usr_queue_state_t usr_queue_out(usr_queue_t *queue_ptr, uint8_t *dest);
usr_queue_state_t usr_queue_in_array(usr_queue_t *queue_ptr, const uint8_t *arr_ptr, uint32_t size);
usr_queue_state_t usr_queue_out_array(usr_queue_t *queue_ptr, uint8_t *arr_ptr, uint32_t size);
usr_queue_state_t usr_queue_out_dump(usr_queue_t *queue_ptr, uint32_t size);

#ifdef __cplusplus
}
#endif
