#include "bsp_uart.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"
#include "rc.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define UART1_TX_BUFF_SIZE 256
#define UART1_RX_BUFF_SIZE 256
#define UART2_TX_BUFF_SIZE 256
#define UART2_RX_BUFF_SIZE 256

uint8_t uart1_queue_tx_buff[UART1_TX_BUFF_SIZE];
uint8_t uart1_rx_buff[UART1_RX_BUFF_SIZE];
uint8_t uart2_queue_tx_buff[UART2_TX_BUFF_SIZE];
uint8_t uart2_rx_buff[UART2_RX_BUFF_SIZE];

uart_tx_t uart1_tx = {
    .queue = {
        .head = 0,
        .tail = 0,
        .size = UART1_TX_BUFF_SIZE,
        .usedSize = 0,
        .queue_buff_ptr = uart1_queue_tx_buff
    },
    .state = UART_IDLE,
    .tx_width = 0
};

uart_rx_t uart1_rx = {
    .parser = NULL,
    .rx_buf = uart1_rx_buff,
    .rx_len = 1,
    .rx_buf_size = UART1_RX_BUFF_SIZE,
    .frame_ready = 0
};

uart_tx_t uart2_tx = {
    .queue = {
        .head = 0,
        .tail = 0,
        .size = UART2_TX_BUFF_SIZE,
        .usedSize = 0,
        .queue_buff_ptr = uart2_queue_tx_buff
    },
    .state = UART_IDLE,
    .tx_width = 0
};

uart_rx_t uart2_rx = {
    .parser = NULL,
    .rx_buf = uart2_rx_buff,
    .rx_len = 1,
    .rx_buf_size = UART2_RX_BUFF_SIZE,
    .frame_ready = 0
};

static void uart1_start(uint16_t size);
static void uart2_start(uint16_t size);

uart_state_t uart1_init(void){
    /* First, enable the receive interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    /* Enable the idle interrupt */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    return UART_SUCCESS;
}

uart_state_t uart1_check(void){
    uart_state_t ret = UART_BUSY;
    usr_queue_out_dump(&uart1_tx.queue, uart1_tx.tx_width);
    if(uart1_tx.queue.usedSize == 0){
        uart1_tx.tx_width = 0;
        uart1_tx.state = UART_IDLE;
        ret = UART_IDLE;
    }else{
        uart1_tx.tx_width = uart1_tx.queue.usedSize;
        uart1_start(uart1_tx.tx_width);
    }
    return ret;
}

uart_state_t uart1_write_byte(uint8_t data){
    uart_state_t ret = UART_SUCCESS;
    if(usr_queue_in(&uart1_tx.queue, &data) == QUEUE_OVERLOAD)
        ret = UART_WARNING_OVERLOAD;

    if(uart1_tx.state != UART_BUSY){
        uart1_tx.state = UART_BUSY;
        uart1_tx.tx_width = 1;
        uart1_start(uart1_tx.tx_width);
    }
    return ret;
}

uart_state_t uart1_write_data(const void *data_ptr, uint16_t size){
    uart_state_t ret = UART_SUCCESS;
    uint16_t tx_width_tmp = 0;
    if(usr_queue_in_array(&uart1_tx.queue, (const uint8_t*)data_ptr, size) == QUEUE_OVERLOAD){
        ret = UART_WARNING_OVERLOAD;
        tx_width_tmp = uart1_tx.queue.size;
    }else tx_width_tmp = size;

    if(uart1_tx.state != UART_BUSY){
        uart1_tx.state = UART_BUSY;
        uart1_tx.tx_width = tx_width_tmp;
        uart1_start(uart1_tx.tx_width);
    }
    return ret;
}

uart_state_t uart2_init(void){
    //
    return UART_SUCCESS;
}

uart_state_t uart2_check(void){
    uart_state_t ret = UART_BUSY;
    usr_queue_out_dump(&uart2_tx.queue, uart2_tx.tx_width);
    if(uart2_tx.queue.usedSize == 0){
        uart2_tx.tx_width = 0;
        uart2_tx.state = UART_IDLE;
        ret = UART_IDLE;
    }
    else{
        uart2_tx.tx_width = uart2_tx.queue.usedSize;
        uart2_start(uart2_tx.tx_width);
    }
    return ret;
}

uart_state_t uart2_write_byte(uint8_t data){
    uart_state_t ret = UART_SUCCESS;
    if(usr_queue_in(&uart2_tx.queue, &data) == QUEUE_OVERLOAD)
        ret = UART_WARNING_OVERLOAD;

    if(uart2_tx.state != UART_BUSY){
        uart2_tx.state = UART_BUSY;
        uart2_tx.tx_width = 1;
        uart2_start(uart2_tx.tx_width);
    }
    return ret;
}

uart_state_t uart2_write_data(const void *data_ptr, uint16_t size){
    uart_state_t ret = UART_SUCCESS;
    uint16_t tx_width_tmp = 0;
    if(usr_queue_in_array(&uart2_tx.queue, (const uint8_t*)data_ptr, size) == QUEUE_OVERLOAD){
        ret = UART_WARNING_OVERLOAD;
        tx_width_tmp = uart2_tx.queue.size;
    }else tx_width_tmp = size;
    
    if(uart2_tx.state != UART_BUSY){
        uart2_tx.state = UART_BUSY;
        uart2_tx.tx_width = tx_width_tmp;
        uart2_start(uart2_tx.tx_width);
    }
    return ret;
}

static void uart1_start(uint16_t size){
    uint16_t contiguous = uart1_tx.queue.size - uart1_tx.queue.head;
    if (size > contiguous){
        size = contiguous;
    }

    uart1_tx.tx_width = size;

    HAL_UART_Transmit_DMA(&huart1, &uart1_queue_tx_buff[uart1_tx.queue.head], size);
}

static void uart2_start(uint16_t size){
    uint16_t contiguous = uart2_tx.queue.size - uart2_tx.queue.head;
    if (size > contiguous){
        size = contiguous;
    }

    uart2_tx.tx_width = size;

    HAL_UART_Transmit_DMA(&huart2, &uart2_queue_tx_buff[uart2_tx.queue.head], size);
}
