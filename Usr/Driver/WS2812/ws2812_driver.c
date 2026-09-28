/**
  ******************************************************************************
  * File Name          : ws2812_driver.c
  * Description        : Code for WS2812 Driver(HAL)
  ******************************************************************************
  */
#include "ws2812_driver.h"

#include <stdint.h>
#include <stdarg.h>

#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "stm32f1xx_hal.h"
#include "tim.h"

#include "usr_delay.h"

/* Usr Definition */
#define MCU_SYS_FREQ 		72000000
#define MCU_SYS_PERIOD 		(1e9f / MCU_SYS_FREQ) //unit: ns
#define ws2812_htim 		htim3
#define WS2812_TIM_CHANNEL 	TIM_CHANNEL_3
#define WS2812_PWM_PSC		0
#define WS2812_PWM_ARR		89

#if (MCU_SYS_FREQ / (WS2812_PWM_PSC+1) / (WS2812_PWM_ARR+1)) != 800000
	#error "WS2812 PWM Freq should be 800KHz"
#endif

#define T0H_PERIOD			300 //unit: ns
#define T1H_PERIOD			800 //unit: ns
#define PIXEL_NUM  			3
#define GRB  				24
#define RES 				300
#define DATA_LEN 			(24*PIXEL_NUM + RES)
#define T0H 				(T0H_PERIOD/MCU_SYS_PERIOD)
#define T1H 				(T1H_PERIOD/MCU_SYS_PERIOD)
#define BRIGHTNESS  		0.5f

static uint16_t data_buf[DATA_LEN]; // Must match DMA transfer width: 16bit

static void ws2812_load(void);
static void ws2812_set(uint16_t *data_ptr);
static void ws2812_set_unit(uint16_t *data_ptr, uint16_t pixel_id);
static void ws2812_parser_rgb(uint16_t *data_ptr, uint32_t rgb);

/**
  * @brief  Set the same color for all lights.
  * @note   None
  * @param  rgb: 0x00000000 ~ 0x00FFFFFF
  * @retval None
  */
ws2812_state_t ws2812_rgb_all(uint32_t rgb){
	uint16_t data[24] = {0};
	ws2812_parser_rgb(data, rgb);
	ws2812_set(data);
	ws2812_load();
	return WS2812_SUCCESS;
}

/**
  * @brief  Set the same color for one light.
  * @note   None
  * @param  rgb: 0x00000000 ~ 0x00FFFFFF
  * @param  pixel_id: 1 ~ PIXEL_NUM
  * @retval None
  */
ws2812_state_t ws2812_rgb_unit(uint32_t rgb, uint16_t pixel_id){
	uint16_t data[24] = {0};
	ws2812_parser_rgb(data, rgb);
	ws2812_set_unit(data, pixel_id);
	ws2812_load();
	return WS2812_SUCCESS;
}

/**
  * @brief  Flow effect: light up each LED one by one in a loop.
  * @note   A delay is applied between each step. The LED index cycles from 1 to PIXEL_NUM.
  * @param  rgb       Color value (0x00000000 ~ 0x00FFFFFF)
  * @param  delay_ms  Delay in milliseconds between steps
  * @retval WS2812_SUCCESS on success, WS2812_WAITING_DELAY if delay is not finished.
  */
ws2812_state_t ws2812_effect_flow(uint32_t rgb, uint16_t delay_ms){
    if(usr_delay_ms(delay_ms) != USR_DELAY_SUCCESS_END) return WS2812_WAITING_DELAY;
	static uint16_t i = 0;
    if(i == PIXEL_NUM) i = 0;
    switch(i++){
        case 0:{
            ws2812_rgb_all(0x00000000);
        }break;
        default:{
            ws2812_rgb_unit(rgb, i);
        }break;
    }
	return WS2812_SUCCESS;
}

/**
  * @brief  Blink effect: all LEDs turn on and off alternately.
  * @note   The on and off durations are specified in milliseconds.
  * @param  rgb      Color value (0x00000000 ~ 0x00FFFFFF)
  * @param  on_ms    Duration in milliseconds for the ON state
  * @param  off_ms   Duration in milliseconds for the OFF state
  * @retval WS2812_SUCCESS on success, WS2812_WAITING_DELAY if delay is not finished.
  */
ws2812_state_t ws2812_effect_blink(uint32_t rgb, uint16_t on_ms, uint16_t off_ms) { 
    static uint8_t i = 0;
    if(i == 0){
        ws2812_rgb_all(0x000000);
        if(usr_delay_ms(off_ms) != USR_DELAY_SUCCESS_END) return WS2812_WAITING_DELAY;
        i = 1;
        return WS2812_SUCCESS;
    }else{
        ws2812_rgb_all(rgb);
        if(usr_delay_ms(on_ms) != USR_DELAY_SUCCESS_END) return WS2812_WAITING_DELAY;
        i = 0;
        return WS2812_SUCCESS;
    }
}

/**
  * @brief  Gradient effect: all LEDs smoothly transition from rgb1 to rgb2.
  * @note   The transition is divided into 100 steps. A delay is applied between each step.
  * @param  rgb1      Start color (0x00000000 ~ 0x00FFFFFF)
  * @param  rgb2      End color (0x00000000 ~ 0x00FFFFFF)
  * @param  delay_ms  Delay in milliseconds between steps
  * @retval WS2812_SUCCESS on success, WS2812_WAITING_DELAY if delay is not finished.
  */
ws2812_state_t ws2812_effect_gradient(uint32_t rgb1, uint32_t rgb2, uint16_t delay_ms){
    if(usr_delay_ms(delay_ms) != USR_DELAY_SUCCESS_END) return WS2812_WAITING_DELAY;
    uint8_t steps = 100u;
    uint8_t r1 = RGB_R(rgb1), g1 = RGB_G(rgb1), b1 = RGB_B(rgb1);
    uint8_t r2 = RGB_R(rgb2), g2 = RGB_G(rgb2), b2 = RGB_B(rgb2);
    static uint16_t i = 0;
    if(i <= steps){
        uint8_t r = (uint8_t)(r1 + (r2 - r1) * i / steps);
        uint8_t g = (uint8_t)(g1 + (g2 - g1) * i / steps);
        uint8_t b = (uint8_t)(b1 + (b2 - b1) * i / steps);
        uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        ws2812_rgb_all(c);
        i++;
    }
    return WS2812_SUCCESS;
}

/**
  * @brief  Convert HSV color to RGB color.
  * @note   h (hue) is 0~359, s (saturation) is 0~255, v (value) is 0~255.
  * @param  h  Hue angle in degrees (0~359)
  * @param  s  Saturation (0~255)
  * @param  v  Value/brightness (0~255)
  * @retval 24-bit RGB color value (0x00RRGGBB)
  */
RGB_Color HSV_ToRGB(uint16_t h, uint8_t s, uint8_t v){
    if (s == 0) {
        return ((uint32_t)v << 16) | ((uint32_t)v << 8) | v;
    }
    
    h = h % 360;
    uint8_t region = h / 60;
    uint8_t rem = (h % 60) * 255 / 60;
    
    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - (s * rem) / 255)) / 255;
    uint8_t t = (v * (255 - (s * (255 - rem)) / 255)) / 255;
    
    uint8_t r, g, b;
    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
  * @brief  Rainbow effect: each LED has a different hue, and the hue rotates over time.
  * @note   The hue difference between adjacent LEDs is 360/PIXEL_NUM degrees.
  *         Brightness is scaled by BRIGHTNESS. A delay is applied between each update.
  * @param  delay_ms  Delay in milliseconds between hue updates
  * @retval WS2812_SUCCESS on success, WS2812_WAITING_DELAY if delay is not finished.
  */
ws2812_state_t ws2812_rainbow(uint16_t delay_ms){
    if(usr_delay_ms(delay_ms) != USR_DELAY_SUCCESS_END) return WS2812_WAITING_DELAY;
    static uint16_t hue = 0;

    uint32_t colors[PIXEL_NUM];

    for (uint16_t i = 0; i < PIXEL_NUM; i++){
        // Each LED differs by 120 degrees
        uint16_t h = (hue + i * 360 / PIXEL_NUM) % 360;

        // HSV -> RGB
        uint32_t color = HSV_ToRGB(h, 255, 255);

        // Brightness scaling
        uint8_t r = RGB_R(color);
        uint8_t g = RGB_G(color);
        uint8_t b = RGB_B(color);

        r = (uint8_t)(r * BRIGHTNESS);
        g = (uint8_t)(g * BRIGHTNESS);
        b = (uint8_t)(b * BRIGHTNESS);

        colors[i] = ((uint32_t)r << 16) |
                    ((uint32_t)g << 8) |
                    b;
    }
    ws2812_rgb_unit(colors[0], 1);
    ws2812_rgb_unit(colors[1], 2);
    ws2812_rgb_unit(colors[2], 3);

    hue = (hue + 3) % 360;
    return WS2812_SUCCESS;
}

/**
  * @brief  Start PWM DMA transfer to send the data buffer to the WS2812 LEDs.
  * @note   None
  * @param  None
  * @retval None
  */
static void ws2812_load(void){
  HAL_TIM_PWM_Start_DMA(&ws2812_htim, WS2812_TIM_CHANNEL, (uint32_t *)data_buf, DATA_LEN);
}

/**
  * @brief  Set the same color for all lights.
  * @note   None
  * @param  Data_ptr: The address of the GRB array that stores color data.
  * @retval None
  */
static void ws2812_set(uint16_t *data_ptr){
	uint16_t i = 0, j = 0;
	for(; i < DATA_LEN; i++){
		if(i < PIXEL_NUM*24){
			if(i % 24 == 0) j = 0;
			data_buf[i] = data_ptr[j++];
		}
		else data_buf[i] = 0;
	}
}

/**
  * @brief  Set the specific color for one of the lights.
  * @note   None
  * @param  Data_ptr: The address of the GRB array that stores color data.
  * @param  pixel_id: 1 ~ PIXEL_NUM
  * @retval None
  */
static void ws2812_set_unit(uint16_t *data_ptr, uint16_t pixel_id){
	uint16_t i = ((pixel_id <= PIXEL_NUM ? pixel_id : PIXEL_NUM) - 1) * 24;
	for(uint16_t j = 0; i < DATA_LEN; i++){
		if(i < PIXEL_NUM*24){
			data_buf[i] = data_ptr[j++];
			if(j > 23){i = PIXEL_NUM*24 - 1; continue;}
		}
		else data_buf[i] = 0;
	}
}

/**
  * @brief  Parse a 24-bit RGB color into a GRB bit pattern for WS2812.
  * @note   None
  * @param  Data_ptr: The address of the GRB array that stores color data.
  * @param  rgb: 0x00000000 ~ 0x00FFFFFF
  * @retval None
  */
static void ws2812_parser_rgb(uint16_t *data_ptr, uint32_t rgb){
	uint16_t i;
	uint8_t R = (rgb & 0x00FF0000) >> 16, G = (rgb & 0x0000FF00) >> 8, B = rgb & 0x000000FF;
	for(i = 0; i < 8; i++){
		data_ptr[i] = ((G & 0x80) ? T1H : T0H);
		data_ptr[i + 8] = ((R & 0x80) ? T1H : T0H);
		data_ptr[i + 16] = ((B & 0x80) ? T1H : T0H);
		G <<= 1, R <<= 1, B <<= 1;
	}
}