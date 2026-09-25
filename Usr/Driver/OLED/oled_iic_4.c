#include "oled_iic_4.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "i2c.h"

#include "usr_queue.h"
#include "oled_font.h"
#include "usr_delay.h"
#include "bsp_i2c.h"

#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDRESS    I2C1_ADDRESS
#define OLED_QUEUE_SIZE I2C1_QUEUE_SIZE
#define OLED_DATA       0x40
#define OLED_CMD        0x00

#define oled_check() i2c1_check()
#define oled_write_byte(x) i2c1_write_byte(x)
#define oled_write_data(x) i2c1_write_data(&(x), sizeof(x))
#define oled_write_ptr(p, n) i2c1_write_data(p, n)

static oled_state_t oled_ctrl_cmd(const uint8_t cmd);
static oled_state_t oled_ctrl_data(const uint8_t data);
static oled_state_t oled_set_cursor(uint8_t page, uint8_t column);

oled_state_t oled_clear(void){  
	uint8_t i, j;
	for (j = 0; j < 8; j++){
		oled_set_cursor(j, 0);
		for(i = 0; i < 128; i++){
			oled_ctrl_data(0x00);
		}
	}
    return OLED_SUCCESS;
}

oled_state_t oled_init(void){
    uint8_t oled_init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x30, 0xA4, 0xA6, 0x8D, 0x14, 0xAF
    };
    uint8_t i, ret = OLED_SUCCESS;
    for(i = 0; i < sizeof(oled_init_cmds); i++){
        ret = oled_ctrl_cmd(oled_init_cmds[i]);
    }
    ret = oled_clear();
    return ret;
}

oled_state_t oled_show_char(uint8_t line, uint8_t column, char font){
    uint8_t i;
    uint8_t ret = OLED_SUCCESS;

    oled_set_cursor((line - 1) * 2, (column - 1) * 8);
    for (i = 0; i < 8; i++){
        if(oled_ctrl_data(OLED_F8x16[font - ' '][i]) == OLED_WARNING_OVERLOAD)
            ret = OLED_WARNING_OVERLOAD;
    }

    oled_set_cursor((line - 1) * 2 + 1, (column - 1) * 8);
    for (i = 0; i < 8; i++){
        if(oled_ctrl_data(OLED_F8x16[font - ' '][i + 8]) == OLED_WARNING_OVERLOAD)
            ret = OLED_WARNING_OVERLOAD;
    }

    return ret;
}

oled_state_t oled_show_string(uint8_t line, uint8_t column, char *font_string){
    uint8_t i;
    uint8_t ret = OLED_SUCCESS;

    for (i = 0; font_string[i] != '\0'; i++){
        if(oled_show_char(line, column + i, font_string[i]) == OLED_WARNING_OVERLOAD)
            ret = OLED_WARNING_OVERLOAD;
    }

    return ret;
}

oled_state_t oled_show_picture(uint8_t line, uint8_t column){
    uint8_t ret = OLED_SUCCESS;

    if( (column >= OLED_WIDTH || line >= OLED_HEIGHT)     ||
        ((uint16_t)column + BMP_WIDTH > OLED_WIDTH)       ||
        ((uint16_t)line + BMP_HEIGHT > OLED_HEIGHT)       ||
        (BMP_HEIGHT % 8 != 0)){
            return OLED_ERROR_OVERFLOW;
        }

    uint32_t j = 0;

    for (uint8_t page = line / 8; page < (line + BMP_HEIGHT) / 8; page++){
        oled_set_cursor(page, column);
        for (uint8_t x = 0; x < BMP_WIDTH; x++){
            if(oled_ctrl_data(BMP[j++]) == OLED_WARNING_OVERLOAD)
                ret = OLED_WARNING_OVERLOAD;
        }
    }

    return ret;
}

oled_state_t oled_show_fonts(uint8_t page, uint8_t column){
    uint8_t ret = OLED_SUCCESS;

    if (column >= OLED_WIDTH || page >= OLED_HEIGHT / 8){
        return OLED_ERROR_OVERFLOW;
    }

    if (page + 1 >= OLED_HEIGHT / 8){
        return OLED_ERROR_OVERFLOW;
    }

    oled_set_cursor(page, column);
    for (uint16_t i = 0; i < sizeof(FONTS); i += 16){
        for (uint8_t j = 0; j < 8; j++){
            if(oled_ctrl_data(FONTS[i + j]) == OLED_WARNING_OVERLOAD)
                ret = OLED_WARNING_OVERLOAD;
        }
    }

    oled_set_cursor(page + 1, column);
    for (uint16_t i = 0; i < sizeof(FONTS); i += 16){
        for (uint8_t j = 8; j < 16; j++){
            if(oled_ctrl_data(FONTS[i + j]) == OLED_WARNING_OVERLOAD)
                ret = OLED_WARNING_OVERLOAD;
        }
    }

    return ret;
}

oled_state_t oled_show_anim(uint8_t page, uint8_t column){
    static int8_t frame = 0, direction = 0;
    uint8_t ret = OLED_SUCCESS;

    if(usr_delay_ms(100) != USR_DELAY_SUCCESS_END) return OLED_WAITING_DELAY;

    if (column + ANIM_WIDTH > OLED_WIDTH)
        return OLED_ERROR_OVERFLOW;

    if (page + 3 > OLED_HEIGHT / 8)
        return OLED_ERROR_OVERFLOW;

    const uint8_t *data = ANIM_20X20[frame];

    for (uint8_t i = 0; i < 3; i++){
        oled_set_cursor(page + i, column);
        for (uint8_t x = 0; x < ANIM_WIDTH; x++){
            if(oled_ctrl_data(data[x * 3 + i]) == OLED_WARNING_OVERLOAD)
                ret = OLED_WARNING_OVERLOAD;
        }
    }

    if(direction == 0){
        frame++;
    }else{
        frame--;
    }

    if (frame >= ANIM_FRAME_COUNT){
        frame = ANIM_FRAME_COUNT - 1;
        direction = 1;
    }else if(frame < 0){
        frame = 0;
        direction = 0;
    }

    return ret;
}

static oled_state_t oled_ctrl_cmd(const uint8_t cmd){
    uint8_t ret = OLED_SUCCESS;
    uint16_t tmp = (cmd << 8) | OLED_CMD;
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}

static oled_state_t oled_ctrl_data(const uint8_t data){
    uint8_t ret = OLED_SUCCESS;
    uint16_t tmp = (data << 8) | OLED_DATA;
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}

static oled_state_t oled_set_cursor(uint8_t page, uint8_t column){
    uint8_t ret = OLED_SUCCESS;
    uint8_t tmp[] = {   OLED_CMD, 0xB0 | page,
                        OLED_CMD, 0x10 | ((column & 0xF0) >> 4),
                        OLED_CMD, 0x00 | (column & 0x0F)};
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}
