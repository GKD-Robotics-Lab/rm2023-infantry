#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;

void buzzer_init(void)
{
    buzzer_off();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    // __HAL_TIM_PRESCALER(&htim4, psc);
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}
void buzzer_off(void)
{
    // __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
