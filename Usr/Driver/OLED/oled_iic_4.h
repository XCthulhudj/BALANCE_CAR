#pragma  once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum{
	OLED_SUCCESS		= 0,
	OLED_ERROR_OVERFLOW	= -1,
	OLED_WAITING_DELAY	= -2
}oled_state_t;

oled_state_t oled_send(void);
oled_state_t oled_clear(void);
oled_state_t oled_init(void);
oled_state_t oled_show_char(uint8_t line, uint8_t column, char font);
oled_state_t oled_show_string(uint8_t line, uint8_t column, char *font_string);
oled_state_t oled_show_picture(uint8_t line, uint8_t column);
oled_state_t oled_show_fonts(uint8_t line, uint8_t column);
oled_state_t oled_show_anim(uint8_t page, uint8_t column);

#ifdef __cplusplus
}
#endif