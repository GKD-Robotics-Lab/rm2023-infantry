/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
//#include "UI.h"
#include "string.h"
#include "tim.h"
#include "remote_control.h"
#include "custom_ui_task.h"


#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define SERVO_OFF_POS   1670
#define SERVO_ON_POS    0620

#define PWM_DETAL_VALUE 10

// 弹舱盖板使用的pwm端口，弹舱盖板的遥控死区
#define COVER_PLATE_PORT 0
#define COVER_RC_DEADBAND 600

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_G
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const RC_ctrl_t *servo_rc;

int servo_state = 0; //0-off, 1-on

// const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          控制弹舱盖板开合
  * @author         Rhine GKD
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void set_servo_on();
void set_servo_off();
uint16_t last_key_state, last_rc_state;

void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
    while(1)
    {
        if(((servo_rc->key.v & KEY_PRESSED_OFFSET_R) && !(last_key_state & KEY_PRESSED_OFFSET_R))
                || ((servo_rc->rc.ch[4] >600) && !(last_rc_state >600)))
        {
            if(servo_state == 0) servo_state = 1;
            else if(servo_state == 1) servo_state = 0;
        }
        last_key_state = servo_rc->key.v;
        last_rc_state = servo_rc->rc.ch[4];

        if(servo_state == 1) 
        {
            set_servo_on();
            UI_Data.cover_state = 1;
        }
        else if(servo_state == 0) 
        {
            set_servo_off();
            UI_Data.cover_state = 0;
        }
        
        osDelay(10);
    }
}

void set_servo_on()
{
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SERVO_ON_POS);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

void set_servo_off()
{
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SERVO_OFF_POS);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}