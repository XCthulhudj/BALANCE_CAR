#include "hc_sr04.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_delay.h"
#include "bsp_it.h"

static void hc_sr04_trigger(void);
static void hc_sr04_echo(void);
static void hc_sr04_echo_irq(void *arg);

static hc_sr04_t hc_sr04 = {
    .echo = {
        .done_sig = 0,
        .period = 0,
        .start_timeStamp = 0,
    },
    .trigger_func = hc_sr04_trigger,
    .echo_func = hc_sr04_echo
};

hc_sr04_t* hc_sr04_init(void){
    DWT_INIT();
    exti_it_hook_register(hc_sr04_echo_irq, 1);
    return &hc_sr04;
}

static void hc_sr04_trigger(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    delay_us_blocking(12);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void hc_sr04_echo(void){
    if(hc_sr04.echo.done_sig == 1){
        hc_sr04.echo.done_sig = 0;
        float us = (float)hc_sr04.echo.period / (SYSTEM_CLOCK / 1000000.0f);
        // 有效回波窗口：约 2cm ~ 400cm 对应 116µs ~ 23.5ms
        if (us > 100.0f && us < 25000.0f) {
            hc_sr04.distance = us * 0.017f;
        } else {
            hc_sr04.distance = -1.0f;  // 标记无效
        }
    }
}

static void hc_sr04_echo_irq(void *arg){
    static uint8_t started = 0;
    uint32_t now = DWT->CYCCNT;

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
        hc_sr04.echo.start_timeStamp = now;
        hc_sr04.echo.done_sig = 0;
        started = 1;
    }else if(started) {
        hc_sr04.echo.period = now - hc_sr04.echo.start_timeStamp;
        hc_sr04.echo.done_sig = 1;
        started = 0;
    }
}
