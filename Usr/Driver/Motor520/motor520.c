#include "motor520.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "usr_algorithm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/**
  * @brief  MAPPING TABLE
  * @note   TIM1_CH4 -> PWMA
  *         TIM1_CH1 -> PWMB
  *         TIM3_CH1 -> M1A
  *         TIM3_CH2 -> M1B
  *         TIM4_CH1 -> M2A
  *         TIM4_CH2 -> M2B
  *         PB14 -> AIN1
  *         PB15 -> AIN2
  *         PB13 -> BIN1
  *         PB12 -> BIN2
  *         0~7200 -> 0~100% DUTY CYCLE
  */
#define MOTOR520_CN1_TIM_START() HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4)
#define MOTOR520_CN2_TIM_START() HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)
#define MOTOR520_CN1_SPEED_SET(x) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (x))
#define MOTOR520_CN2_SPEED_SET(x) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (x))
#define MOTOR520_CN1_ENCODER_START() do{ \
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); \
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); \
}while(0)
#define MOTOR520_CN2_ENCODER_START() do{ \
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); \
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); \
}while(0)
#define MOTOR520_CN1_ENCODER_GET(x) do{ \
    (x) = (int16_t)__HAL_TIM_GET_COUNTER(&htim3); \
    __HAL_TIM_SET_COUNTER(&htim3, 0); \
}while(0)
#define MOTOR520_CN2_ENCODER_GET(x) do{ \
    (x) = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); \
    __HAL_TIM_SET_COUNTER(&htim4, 0); \
}while(0)
#define MOTOR520_CN1_DIR_P() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); \
}while(0)
#define MOTOR520_CN1_DIR_N() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); \
}while(0)
#define MOTOR520_CN1_BRAKE() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); \
}while(0)
#define MOTOR520_CN2_DIR_P() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); \
}while(0)
#define MOTOR520_CN2_DIR_N() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); \
}while(0)
#define MOTOR520_CN2_BRAKE() do{ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); \
}while(0)

#define SAMPLING_PERIOD 0.02f

motor520_t motor520;

motor520_t* motor520_init(void){
    MOTOR520_CN2_TIM_START();
    MOTOR520_CN1_TIM_START();
    MOTOR520_CN1_ENCODER_START();
    MOTOR520_CN2_ENCODER_START();
    MOTOR520_CN1_SPEED_SET(0);
    MOTOR520_CN2_SPEED_SET(0);
    MOTOR520_CN1_DIR_P();
    MOTOR520_CN2_DIR_N();
    return &motor520;
}

motor520_state_t motor520_update(void){
    MOTOR520_CN1_ENCODER_GET(motor520.cn1_enc);
    MOTOR520_CN2_ENCODER_GET(motor520.cn2_enc);

    return MOTOR520_OK;
}

motor520_state_t motor520_rpm_load(void){

    if(motor520.cn1_set > 0){
        MOTOR520_CN1_DIR_P();
    }else{
        MOTOR520_CN1_DIR_N();
    }

    if(motor520.cn2_set > 0){
        MOTOR520_CN2_DIR_P();
    }else{
        MOTOR520_CN2_DIR_N();
    }

    MOTOR520_CN1_SPEED_SET(ABS(motor520.cn1_set));
    MOTOR520_CN2_SPEED_SET(ABS(motor520.cn2_set));
    return MOTOR520_OK;
}
