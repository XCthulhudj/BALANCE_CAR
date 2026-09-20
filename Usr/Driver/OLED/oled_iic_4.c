#include "oled_iic_4.h"

#include <stdint.h>
#include <stdarg.h>

#include "stm32f1xx_hal.h"
#include "i2c.h"

#include "usr_queue.h"
#include "oled_font.h"
#include "usr_delay.h"

#define OLED_ADDRESS 0x78 //The I2C Slave Address
#define OLED_QUEUE_SIZE 256
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

const static uint8_t oled_data = 0x40;
const static uint8_t oled_cmd = 0x00;
uint8_t oled_queue_buff[OLED_QUEUE_SIZE];

usr_queue_t oled_queue = {
    .head = 0,
    .tail = 0,
    .size = OLED_QUEUE_SIZE,
    .queue_buff_ptr = oled_queue_buff
};

static void oled_start(void);
static void oled_write_data(const uint8_t data);
static void oled_write_cmd(const uint8_t cmd);
static void oled_set_cursor(uint8_t Y, uint8_t X);

oled_state_t oled_send(void){
    static uint8_t dump = 0;
    usr_queue_out(&oled_queue, &dump);
    usr_queue_out(&oled_queue, &dump);
    oled_start();
    return OLED_SUCCESS;
}

oled_state_t oled_clear(void){  
	uint8_t i, j;
	for (j = 0; j < 8; j++){
		oled_set_cursor(j, 0);
		for(i = 0; i < 128; i++){
			oled_write_data(0x00);
		}
	}
    return OLED_SUCCESS;
}

oled_state_t oled_init(void){
    oled_write_cmd(0xAE);
    oled_write_cmd(0xD5);
    oled_write_cmd(0x80);
    oled_write_cmd(0xA8);
    oled_write_cmd(0x3F);
    oled_write_cmd(0xD3);
    oled_write_cmd(0x00);
    oled_write_cmd(0x40);
    oled_write_cmd(0xA1);
    oled_write_cmd(0xC8);
    oled_write_cmd(0xDA);
    oled_write_cmd(0x12);
    oled_write_cmd(0x81);
    oled_write_cmd(0xCF);
    oled_write_cmd(0xD9);
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDB);
    oled_write_cmd(0x30);
    oled_write_cmd(0xA4);
    oled_write_cmd(0xA6);
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);
    oled_clear();
    return OLED_SUCCESS;
}

oled_state_t oled_show_char(uint8_t line, uint8_t column, char font){      	
	uint8_t i;
	oled_set_cursor((line - 1) * 2, (column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		oled_write_data(OLED_F8x16[font - ' '][i]);			//显示上半部分内容
	}
	oled_set_cursor((line - 1) * 2 + 1, (column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		oled_write_data(OLED_F8x16[font - ' '][i + 8]);		//显示下半部分内容
	}
    return OLED_SUCCESS;
}

oled_state_t oled_show_string(uint8_t line, uint8_t column, char *font_string){
	uint8_t i;
	for (i = 0; font_string[i] != '\0'; i++){
		oled_show_char(line, column + i, font_string[i]);
	}
    return OLED_SUCCESS;
}

oled_state_t oled_show_picture(uint8_t line, uint8_t column){
    if( (column >= OLED_WIDTH || line >= OLED_HEIGHT)     || 
        ((uint16_t)column + BMP_WIDTH > OLED_WIDTH)     ||
        ((uint16_t)line + BMP_HEIGHT > OLED_HEIGHT)   ||
        (BMP_HEIGHT % 8 != 0)){
            return OLED_ERROR_OVERFLOW;
        }

    uint32_t j = 0;

    for (uint8_t page = line / 8; page < (line + BMP_HEIGHT) / 8; page++){
        oled_set_cursor(page, column);
        for (uint8_t x = 0; x < BMP_WIDTH; x++){
            oled_write_data(BMP[j++]);
        }
    }

    return OLED_SUCCESS;
}

oled_state_t oled_show_fonts(uint8_t page, uint8_t column)
{
    if (column >= OLED_WIDTH || page >= OLED_HEIGHT / 8){
        return OLED_ERROR_OVERFLOW;
    }

    // 8×16 字符，需要上下两个 Page
    if (page + 1 >= OLED_HEIGHT / 8){
        return OLED_ERROR_OVERFLOW;
    }

    // 第一页
    oled_set_cursor(page, column);

    for (uint16_t i = 0; i < sizeof(FONTS); i += 16){
        for (uint8_t j = 0; j < 8; j++)
        {
            oled_write_data(FONTS[i + j]);
        }
    }

    // 第二页
    oled_set_cursor(page + 1, column);

    for (uint16_t i = 0; i < sizeof(FONTS); i += 16)
    {
        for (uint8_t j = 8; j < 16; j++)
        {
            oled_write_data(FONTS[i + j]);
        }
    }

    return OLED_SUCCESS;
}

oled_state_t oled_show_anim(uint8_t page, uint8_t column){
    static int8_t frame = 0, direction = 0;

    if(usr_delay_ms(100) != USR_DELAY_SUCCESS_END) return OLED_WAITING_DELAY;

    if (column + ANIM_WIDTH > OLED_WIDTH)
        return OLED_ERROR_OVERFLOW;

    if (page + 3 > OLED_HEIGHT / 8)
        return OLED_ERROR_OVERFLOW;

    const uint8_t *data = ANIM_20X20[frame];

    for (uint8_t i = 0; i < 3; i++)
    {
        oled_set_cursor(page + i, column);

        for (uint8_t x = 0; x < ANIM_WIDTH; x++)
        {
            oled_write_data(data[x * 3 + i]);
        }
    }
    if(direction == 0){
        frame++;
    }else{
        frame--;
    }
    if (frame >= ANIM_FRAME_COUNT){
        frame = ANIM_FRAME_COUNT-1;
        direction = 1;
    }else if(frame < 0){
        frame = 0;
        direction = 0;
    }

    return OLED_SUCCESS;
}

static void oled_start(void){
    if(oled_queue.head != oled_queue.tail){
        HAL_I2C_Master_Transmit_DMA(&hi2c1, OLED_ADDRESS, &oled_queue_buff[oled_queue.head],2);
    }
}

static void oled_write_data(const uint8_t data){
    usr_queue_in(&oled_queue, &oled_data);
    usr_queue_in(&oled_queue, &data);
    oled_start();
}

static void oled_write_cmd(const uint8_t cmd){
    usr_queue_in(&oled_queue, &oled_cmd);
    usr_queue_in(&oled_queue, &cmd);
    oled_start();
}

static void oled_set_cursor(uint8_t Y, uint8_t X){
    oled_write_cmd(0xB0 | Y);
    oled_write_cmd(0x10 | ((X & 0xF0) >> 4));
    oled_write_cmd(0x00 | (X & 0x0F));
}
