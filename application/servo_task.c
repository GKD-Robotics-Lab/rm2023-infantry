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

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

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
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
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

void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
    //static String_Data change_white, change_black;
    //Char_Draw(&change_white, "001", UI_Graph_ADD, 9, UI_Color_White, 2, 8, 5, 200, 200, "cover");
    //Char_Draw(&change_white, "002", UI_Graph_ADD, 9, UI_Color_Black, 2, 8, 5, 200, 200, "cover");

//    Graph_Data change_white, change_black;;
//    Line_Draw(&change_white, "001", UI_Graph_Change, 9, UI_Color_White, 3, 200, 200, 250, 200);
//    Line_Draw(&change_black, "001", UI_Graph_Change, 9, UI_Color_Black, 3, 200, 200, 250, 200);
//    memset(&change_white, 0, sizeof(change_white));
//    memset(&change_black, 0, sizeof(change_black));

    while(1)
    {

        static int16_t last_key_mode = 0;
        static int8_t cover_mode = 0, this_mode = 0;;
        static uint32_t last_offtick = 0;
        if(servo_rc->key.v & SERVO1_ADD_PWM_KEY){
            if(xTaskGetTickCount() - last_offtick >= 1000){
                cover_mode = 1;
                this_mode = 1;
            }
            else{
                this_mode = 0;
            }
        }
        else{
            if(last_key_mode && !this_mode){
                cover_mode = 0;
            }
            last_offtick = xTaskGetTickCount();
        }
        last_key_mode = (servo_rc->key.v & SERVO1_ADD_PWM_KEY);
        if(servo_rc->rc.ch[COVER_PLATE_CHANNEL] > COVER_RC_DEADBAND)
        {
            cover_mode = 1;
        }

        else if(servo_rc->rc.ch[COVER_PLATE_CHANNEL] < -COVER_RC_DEADBAND)
        {
            cover_mode = 0;
        }

        servo_pwm[COVER_PLATE_PORT] = cover_mode ? SERVO_MAX_PWM : SERVO_MIN_PWM;

        if(servo_pwm[COVER_PLATE_PORT] < SERVO_MIN_PWM)
        {
            servo_pwm[COVER_PLATE_PORT] = SERVO_MIN_PWM;
        }
        else if(servo_pwm[COVER_PLATE_PORT] > SERVO_MAX_PWM)
        {
            servo_pwm[COVER_PLATE_PORT] = SERVO_MAX_PWM;
        }
        servo_pwm_set(servo_pwm[COVER_PLATE_PORT], COVER_PLATE_PORT);
        /*
    for(uint8_t i = 0; i < 4; i++)
    {

        if( (servo_rc->key.v & SERVO_MINUS_PWM_KEY) && (servo_rc->key.v & servo_key[i]))
        {
            servo_pwm[i] -= PWM_DETAL_VALUE;
        }
        else if(servo_rc->key.v & servo_key[i])
        {
            servo_pwm[i] += PWM_DETAL_VALUE;
        }

        //limit the pwm
       //限制pwm
        if(servo_pwm[i] < SERVO_MIN_PWM)
        {
            servo_pwm[i] = SERVO_MIN_PWM;
        }
        else if(servo_pwm[i] > SERVO_MAX_PWM)
        {
            servo_pwm[i] = SERVO_MAX_PWM;
        }

        servo_pwm_set(servo_pwm[i], i);
    }
        */
        osDelay(10);
    }
}

