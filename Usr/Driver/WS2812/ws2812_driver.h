#pragma  once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RGB_R(c)    (((c) >> 16) & 0xFF)
#define RGB_G(c)    (((c) >> 8)  & 0xFF)
#define RGB_B(c)    ((c) & 0xFF)

#define RGB_BLACK       0x000000
#define RGB_WHITE       0xFFFFFF
#define RGB_RED         0xFF0000
#define RGB_GREEN       0x00FF00
#define RGB_BLUE        0x0000FF
#define RGB_YELLOW      0xFFFF00
#define RGB_CYAN        0x00FFFF
#define RGB_MAGENTA     0xFF00FF
#define RGB_ORANGE      0xFFA500
#define RGB_PURPLE      0x800080
#define RGB_PINK        0xFFC0CB
#define RGB_GRAY        0x808080

typedef enum{
    WS2812_SUCCESS              =  0,
    WS2812_ERROR_INVALID_PARAM  = -1,
    WS2812_ERROR_MEMORY         = -2,
    WS2812_ERROR_HARDWARE       = -3,
    WS2812_WAITING_DELAY        = -4
}ws2812_state_t;

typedef uint32_t RGB_Color;

ws2812_state_t ws2812_rgb_all(uint32_t rgb);
ws2812_state_t ws2812_rgb_unit(uint32_t rgb, uint16_t pixel_id);
ws2812_state_t ws2812_effect_flow(uint32_t rgb, uint16_t delay_ms);
ws2812_state_t ws2812_effect_blink(uint32_t rgb, uint16_t on_ms, uint16_t off_ms);
ws2812_state_t ws2812_effect_gradient(uint32_t rgb1, uint32_t rgb2, uint16_t steps, uint16_t delay_ms);
ws2812_state_t ws2812_rainbow(uint16_t delay_ms);

RGB_Color HSV_ToRGB(uint16_t h, uint8_t s, uint8_t v);

#ifdef __cplusplus
}
#endif
