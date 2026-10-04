#include "bsp_i2c.h"

#include <stdint.h>
#include <stdarg.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static uint8_t i2c1_tx_queue_buff[I2C1_TX_QUEUE_SIZE];

static i2c_tx_t i2c1_tx = {
    .queue = {
        .head = 0,
        .tail = 0,
        .size = I2C1_TX_QUEUE_SIZE,
        .usedSize = 0,
        .queue_buff_ptr = i2c1_tx_queue_buff
    },
    .state = I2C_IDLE,
    .tx_width = 0
};

static bsp_i2c_t i2c2 = {
    .irq_parser = NULL,
    .buff_ptr = NULL,
    .dev_id = 0,
};

static void i2c1_tx_start(uint16_t size);

i2c_state_t i2c1_write_byte(uint8_t data){
    i2c_state_t ret = I2C_OK;
    if(usr_queue_in(&i2c1_tx.queue, &data) == QUEUE_OVERLOAD)
        ret = I2C_WARNING_OVERLOAD;

    if(i2c1_tx.state == I2C_IDLE){
        i2c1_tx.state = I2C_BUSY;
        i2c1_tx.tx_width = 1;
        i2c1_tx_start(i2c1_tx.tx_width);
    }
    return ret;
}

i2c_state_t i2c1_write_data(const void *data_ptr, uint16_t size){
    i2c_state_t ret = I2C_OK;
    uint16_t tx_width_tmp = 0;
    if(usr_queue_in_array(&i2c1_tx.queue, (const uint8_t*)data_ptr, size) == QUEUE_OVERLOAD){
        ret = I2C_WARNING_OVERLOAD;
        tx_width_tmp = i2c1_tx.queue.size;
    }else tx_width_tmp = size;

    if(i2c1_tx.state == I2C_IDLE){
        i2c1_tx.state = I2C_BUSY;
        i2c1_tx.tx_width = tx_width_tmp;
        i2c1_tx_start(i2c1_tx.tx_width);
    }
    return ret;
}

i2c_state_t i2c1_tx_handler(void){
    i2c_state_t ret = I2C_BUSY;
    usr_queue_out_none(&i2c1_tx.queue, i2c1_tx.tx_width);
    if(i2c1_tx.queue.usedSize == 0){
        i2c1_tx.tx_width = 0;
        i2c1_tx.state = I2C_IDLE;
        ret = I2C_IDLE;
    }else{
        i2c1_tx.tx_width = i2c1_tx.queue.usedSize;
        i2c1_tx_start(i2c1_tx.tx_width);
    }
    return ret;
}

i2c_state_t i2c2_write_byte_memAddSize_8bit(uint16_t mem_address, uint8_t data){
    HAL_I2C_Mem_Write(&hi2c2, i2c2.dev_id, mem_address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return I2C_OK;
}

i2c_state_t i2c2_read_start_memAddSize_8bit(uint16_t mem_address, uint16_t size){
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(&hi2c2, i2c2.dev_id, mem_address, I2C_MEMADD_SIZE_8BIT, i2c2.buff_ptr, size);
    switch(ret){
        case HAL_OK: return I2C_OK;
        case HAL_ERROR: return I2C_ERROR;
        case HAL_BUSY: return I2C_BUSY;
        case HAL_TIMEOUT: return I2C_TIMEOUT;
        default: return I2C_OK;
    }
}

i2c_state_t i2c2_read_byte_memAddSize_8bit_blocking(uint16_t devAddr, uint8_t memAddr, uint8_t *data){
    return HAL_I2C_Mem_Read(&hi2c2, i2c2.dev_id, memAddr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

i2c_state_t i2c2_hook_register(i2c_event_hook_t func_parser, uint8_t *buff_ptr, uint8_t dev_id){
    i2c2.irq_parser = func_parser;
    i2c2.buff_ptr = buff_ptr;
    i2c2.dev_id = dev_id;
    return I2C_OK;
}

i2c_state_t i2c2_rx_parser_handler(void){
    if(i2c2.irq_parser == NULL) return I2C_WARNING_NULL;
    i2c2.irq_parser(NULL);
    return I2C_OK;
}

static void i2c1_tx_start(uint16_t size){
    uint16_t contiguous = i2c1_tx.queue.size - i2c1_tx.queue.head;
    if(size > contiguous){
        size = contiguous;
    }

    i2c1_tx.tx_width = size;

    HAL_I2C_Master_Transmit_DMA(&hi2c1, I2C1_ADDRESS, &i2c1_tx_queue_buff[i2c1_tx.queue.head],size);
}
