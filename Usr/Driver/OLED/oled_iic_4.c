#include "oled_iic_4.h"

#include <stdint.h>
#include <string.h>

#include "bsp_i2c.h"
#include "oled_font.h"
#include "usr_delay.h"

#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDRESS    I2C1_ADDRESS
#define OLED_QUEUE_SIZE I2C1_TX_QUEUE_SIZE
#define OLED_DATA       0x40
#define OLED_CMD        0x00

#define oled_write_byte(x) i2c1_write_byte(x)
#define oled_write_data(x) i2c1_write_data(&(x), sizeof(x))
#define oled_write_ptr(p, n) i2c1_write_data(p, n)

static inline oled_state_t oled_ctrl_cmd(const uint8_t cmd);
static inline oled_state_t oled_ctrl_data(const uint8_t data);
static inline oled_state_t oled_set_cursor(uint8_t page, uint8_t column);

/**
  * @brief  Clear the entire OLED screen.
  * @note   Writes 0x00 to all 8 pages and 128 columns.
  * @param  None
  * @retval OLED_SUCCESS on success, OLED_WARNING_OVERLOAD if queue overflows.
  */
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

/**
  * @brief  Initialize the OLED display.
  * @note   Sends a fixed sequence of commands for SSD1306, then clears the screen.
  * @param  None
  * @retval OLED_SUCCESS on success, OLED_WARNING_OVERLOAD if any command fails.
  */
oled_state_t oled_init(void){
    uint8_t oled_init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x30, 0xA4, 0xA6, 0x8D, 0x14, 0xAF
    };

    oled_state_t ret = OLED_SUCCESS;
    uint8_t i;
    
    for(i = 0; i < sizeof(oled_init_cmds); i++){
        ret = oled_ctrl_cmd(oled_init_cmds[i]);
    }
    ret = oled_clear();
    return ret;
}

/**
  * @brief  Display a single character at the specified position.
  * @note   Uses 8x16 font. 'line' is 1-based (1..4), 'column' is 1-based (1..16).
  *         Each character occupies two pages (upper and lower halves).
  * @param  line    Line number (1..OLED_HEIGHT/16)
  * @param  column  Column number (1..OLED_WIDTH/8)
  * @param  font    ASCII character to display
  * @retval OLED_SUCCESS on success, OLED_ERROR_OVERFLOW if out of bounds,
  *         OLED_WARNING_OVERLOAD if queue overflows.
  */
oled_state_t oled_show_char(uint8_t line, uint8_t column, char font){
    if(line > OLED_HEIGHT/16 || column > OLED_WIDTH/8){
        return OLED_ERROR_OVERFLOW;
    }

    oled_state_t ret = OLED_SUCCESS;
    uint8_t i;

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

/**
  * @brief  Display a null-terminated string starting at the given position.
  * @note   Calls oled_show_char for each character until '\0'.
  * @param  line         Starting line (1-based)
  * @param  column       Starting column (1-based)
  * @param  font_string  Pointer to the string to display
  * @retval OLED_SUCCESS on success, OLED_ERROR_OVERFLOW if any character overflows,
  *         OLED_WARNING_OVERLOAD if queue overflows.
  */
oled_state_t oled_show_string(uint8_t line, uint8_t column, char *font_string){
    oled_state_t ret = OLED_SUCCESS;
    uint8_t i;

    for (i = 0; font_string[i] != '\0'; i++){
        ret = oled_show_char(line, column + i, font_string[i]);
        if(ret == OLED_ERROR_OVERFLOW) return ret;
    }

    return ret;
}

/**
  * @brief  Display a bitmap image at the specified pixel coordinates.
  * @note   The bitmap dimensions are BMP_WIDTH and BMP_HEIGHT (defined elsewhere).
  *         BMP_HEIGHT must be a multiple of 8.
  * @param  line_pixel    Starting Y pixel (0..OLED_HEIGHT-1)
  * @param  column_pixel  Starting X pixel (0..OLED_WIDTH-1)
  * @retval OLED_SUCCESS on success, OLED_ERROR_OVERFLOW if out of bounds,
  *         OLED_WARNING_OVERLOAD if queue overflows.
  */
oled_state_t oled_show_picture(uint8_t line_pixel, uint8_t column_pixel){
    oled_state_t ret = OLED_SUCCESS;

    if( (column_pixel >= OLED_WIDTH || line_pixel >= OLED_HEIGHT)   ||
        ((uint16_t)column_pixel + BMP_WIDTH > OLED_WIDTH)           ||
        ((uint16_t)line_pixel + BMP_HEIGHT > OLED_HEIGHT)           ||
        (BMP_HEIGHT % 8 != 0)){
            return OLED_ERROR_OVERFLOW;
        }

    uint32_t j = 0;

    for (uint8_t page = line_pixel / 8; page < (line_pixel + BMP_HEIGHT) / 8; page++){
        oled_set_cursor(page, column_pixel);
        for (uint8_t x = 0; x < BMP_WIDTH; x++){
            if(oled_ctrl_data(BMP[j++]) == OLED_WARNING_OVERLOAD)
                ret = OLED_WARNING_OVERLOAD;
        }
    }

    return ret;
}

/**
  * @brief  Display a custom font array (FONTS) spanning two pages.
  * @note   FONTS is expected to contain 16 bytes per character (8 for upper page, 8 for lower).
  *         The function writes all characters in FONTS starting at the given page and column.
  * @param  page    Starting page (0..OLED_HEIGHT/8 - 2)
  * @param  column  Starting column (0..OLED_WIDTH-1)
  * @retval OLED_SUCCESS on success, OLED_ERROR_OVERFLOW if out of bounds,
  *         OLED_WARNING_OVERLOAD if queue overflows.
  */
oled_state_t oled_show_fonts(uint8_t page, uint8_t column){
    oled_state_t ret = OLED_SUCCESS;

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

/**
  * @brief  Display an animation frame from ANIM_20X20.
  * @note   The animation has ANIM_FRAME_COUNT frames, each 20x20 pixels (3 pages high).
  *         A 100ms delay is applied between frames. The frame index oscillates between
  *         0 and ANIM_FRAME_COUNT-1.
  * @param  page    Starting page (0..OLED_HEIGHT/8 - 3)
  * @param  column  Starting column (0..OLED_WIDTH - ANIM_WIDTH)
  * @retval OLED_SUCCESS on success, OLED_ERROR_OVERFLOW if out of bounds,
  *         OLED_WAITING_DELAY if delay not finished, OLED_WARNING_OVERLOAD if queue overflows.
  */
oled_state_t oled_show_anim(uint8_t page, uint8_t column){
    static int8_t frame = 0, direction = 0;
    oled_state_t ret = OLED_SUCCESS;

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

/**
  * @brief  Send a command byte to the OLED.
  * @note   Prepends the control byte OLED_CMD (0x00).
  * @param  cmd  Command byte to send
  * @retval OLED_SUCCESS on success, OLED_WARNING_OVERLOAD if queue overflows.
  */
static inline oled_state_t oled_ctrl_cmd(const uint8_t cmd){
    oled_state_t ret = OLED_SUCCESS;
    uint16_t tmp = (cmd << 8) | OLED_CMD;
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}

/**
  * @brief  Send a data byte to the OLED.
  * @note   Prepends the control byte OLED_DATA (0x40).
  * @param  data  Data byte to send
  * @retval OLED_SUCCESS on success, OLED_WARNING_OVERLOAD if queue overflows.
  */
static inline oled_state_t oled_ctrl_data(const uint8_t data){
    oled_state_t ret = OLED_SUCCESS;
    uint16_t tmp = (data << 8) | OLED_DATA;
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}

/**
  * @brief  Set the OLED cursor to a specific page and column.
  * @note   Sends three commands: set page address, set higher column address,
  *         and set lower column address.
  * @param  page    Page number (0..7)
  * @param  column  Column number (0..127)
  * @retval OLED_SUCCESS on success, OLED_WARNING_OVERLOAD if queue overflows.
  */
static inline oled_state_t oled_set_cursor(uint8_t page, uint8_t column){
    oled_state_t ret = OLED_SUCCESS;
    uint8_t tmp[] = {   OLED_CMD, 0xB0 | page,
                        OLED_CMD, 0x10 | ((column & 0xF0) >> 4),
                        OLED_CMD, 0x00 | (column & 0x0F)};
    if(oled_write_data(tmp) == I2C_WARNING_OVERLOAD)
        ret = OLED_WARNING_OVERLOAD;
    return ret;
}