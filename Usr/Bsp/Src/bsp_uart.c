#include "bsp_uart.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_queue.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define UART1_BUFF_SIZE 256
#define UART2_BUFF_SIZE 256

uint8_t uart1_queue_buff[UART1_BUFF_SIZE];
uint8_t uart2_queue_buff[UART2_BUFF_SIZE];

usr_queue_t uart1_queue = {
    .head = 0,
    .tail = 0,
    .size = UART1_BUFF_SIZE,
    .queue_buff_ptr = uart1_queue_buff
};

usr_queue_t uart2_queue = {
    .head = 0,
    .tail = 0,
    .size = UART2_BUFF_SIZE,
    .queue_buff_ptr = uart2_queue_buff
};

static void uart1_start(void);
static void uart2_start(void);

uart_state_t uart1_send(void){
    static uint8_t dump = 0;
    usr_queue_out(&uart1_queue, &dump);
    uart1_start();
    return UART_SUCCESS;
}

uart_state_t uart1_write_byte(uint8_t data){
    usr_queue_in(&uart1_queue, &data);
    uart1_start();
    return UART_SUCCESS;
}

uart_state_t uart1_write_arr(const uint8_t *arr, uint16_t size){
    usr_queue_in_array(&uart1_queue, arr, size);
    uart1_start();
    return UART_SUCCESS;
}

static void uart1_start(void){
    if(uart1_queue.head != uart1_queue.tail){
        HAL_UART_Transmit_DMA(&huart1, &uart1_queue_buff[uart1_queue.head], 1);
    }
}

uart_state_t uart2_send(void){
    static uint8_t dump = 0;
    usr_queue_out(&uart2_queue, &dump);
    uart2_start();
    return UART_SUCCESS;
}

uart_state_t uart2_write_byte(uint8_t data){
    usr_queue_in(&uart2_queue, &data);
    uart2_start();
    return UART_SUCCESS;
}

uart_state_t uart2_write_arr(const uint8_t *arr, uint16_t size){
    usr_queue_in_array(&uart2_queue, arr, size);
    uart2_start();
    return UART_SUCCESS;
}

static void uart2_start(void){
    if(uart2_queue.head != uart2_queue.tail){
        HAL_UART_Transmit_DMA(&huart2, &uart2_queue_buff[uart2_queue.head], 1);
    }
}
