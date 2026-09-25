#include "bsp_i2c.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "i2c.h"

#include "usr_queue.h"

uint8_t i2c1_tx_queue_buff[I2C1_QUEUE_SIZE];

i2c_tx_t i2c1_tx = {
    .queue = {
        .head = 0,
        .tail = 0,
        .size = I2C1_QUEUE_SIZE,
        .usedSize = 0,
        .queue_buff_ptr = i2c1_tx_queue_buff
    },
    .state = I2C_IDLE,
    .tx_width = 0
};

static void i2c1_start(uint16_t size);

i2c_state_t i2c1_check(void){
    i2c_state_t ret = I2C_BUSY;
    usr_queue_out_dump(&i2c1_tx.queue, i2c1_tx.tx_width);
    if(i2c1_tx.queue.usedSize == 0){
        i2c1_tx.tx_width = 0;
        i2c1_tx.state = I2C_IDLE;
        ret = I2C_IDLE;
    }else{
        i2c1_tx.tx_width = i2c1_tx.queue.usedSize;
        i2c1_start(i2c1_tx.tx_width);
    }
    return ret;
}

i2c_state_t i2c1_write_byte(uint8_t data){
    i2c_state_t ret = I2C_SUCCESS;
    if(usr_queue_in(&i2c1_tx.queue, &data))
        ret = I2C_WARNING_OVERLOAD;

    if(i2c1_tx.state != I2C_BUSY){
        i2c1_tx.state = I2C_BUSY;
        i2c1_tx.tx_width = 1;
        i2c1_start(i2c1_tx.tx_width);
    }
    return ret;
}

i2c_state_t i2c1_write_data(const void *data_ptr, uint16_t size){
    i2c_state_t ret = I2C_SUCCESS;
    uint16_t tx_width_tmp = 0;
    if(usr_queue_in_array(&i2c1_tx.queue, (const uint8_t*)data_ptr, size) == QUEUE_OVERLOAD){
        ret = I2C_WARNING_OVERLOAD;
        tx_width_tmp = i2c1_tx.queue.size;
    }else tx_width_tmp = size;

    if(i2c1_tx.state != I2C_BUSY){
        i2c1_tx.state = I2C_BUSY;
        i2c1_tx.tx_width = tx_width_tmp;
        i2c1_start(i2c1_tx.tx_width);
    }
    return ret;
}

static void i2c1_start(uint16_t size){
    uint16_t contiguous = i2c1_tx.queue.size - i2c1_tx.queue.head;
    if(size > contiguous){
        size = contiguous;
    }

    i2c1_tx.tx_width = size;

    HAL_I2C_Master_Transmit_DMA(&hi2c1, I2C1_ADDRESS, &i2c1_tx_queue_buff[i2c1_tx.queue.head],size);
}
