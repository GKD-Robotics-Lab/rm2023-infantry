#include "bsp_buzzer.h"
#include "main.h"
#include "config.h"

extern TIM_HandleTypeDef htim4;

void buzzer_init(void)
{
    buzzer_off();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
 * @brief 打开蜂鸣器
 *
 * @param psc 对应频率 0-65535
 * @param pwm 对应响度 0-65535
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
#ifdef BUZZER_ON
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
#endif
}

void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
