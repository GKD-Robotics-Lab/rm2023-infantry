/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"

#include "CAN_receive.h"
#include "UI.h"
#include "arm_math.h"
#include "bsp_fric.h"
#include "bsp_laser.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "main.h"
#include "pid.h"
#include "referee.h"
#include "user_lib.h"

#define shoot_fric1_on(pwm) fric1_on((pwm))  // 摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm))  // 摩擦轮2pwm宏定义
#define shoot_fric_off()    fric_off()       // 关闭两个摩擦轮

#define shoot_laser_on()  laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off()  // 激光关闭宏定义
// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
 * @brief
 * 射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

/**
 * @brief          初始化"gimbal_CV"变量
 * @param[out]     init: 新增了init_CV
 * @retval         none
 */
static void CV_init(shoot_control_t *init);

shoot_control_t shoot_control;  // 射击数据

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = { TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD };
    shoot_control.shoot_mode = SHOOT_STOP;
    // 遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    // 电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    // 初始化PID
    PID_init(
        &shoot_control.trigger_motor_pid,
        PID_POSITION,
        Trigger_speed_pid,
        TRIGGER_READY_PID_MAX_OUT,
        TRIGGER_READY_PID_MAX_IOUT);
    // 更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;

    CV_init(&shoot_control);
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
int16_t shoot_control_loop(gimbal_control_t *gimbal_shoot)
{
    // 状态机函数用于判断并更新状态，shoot_control_loop用于根据状态进行控制操作
    shoot_set_mode();         // 设置状态机
    shoot_feedback_update();  // 更新数据

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        // 设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        // 设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)  // 状态<开始送弹> (官步逻辑)
    {
        /* 官方 */
        //        if(shoot_control.key == SWITCH_TRIGGER_OFF)  // 继续送弹
        //        {
        //            //设置拨弹轮的拨动速度,并开启堵转反转处理
        //            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
        //            trigger_motor_turn_back();
        //        }
        //        else
        //        // 停止送弹
        //        {
        //            shoot_control.trigger_speed_set = 0.0f;
        //            shoot_control.speed_set = 0.0f;
        //        }
        //        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        //        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        /* 修改 */
        // 这个判断体只会进入一次，之后就进入SHOOT_READY
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.speed_set = 0.0f;
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)  // 状态<可以射击>
    {                                                  // 等待进入SHOOT_BULLET状态的指令
        // 设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)  // 状态<开始射击-点射>
    {
        /* 官方 */
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
        /* 修改 */
        //				static uint16_t i=0;
        //        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        //        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        //				shoot_control.trigger_speed_set = TRIGGER_SPEED;
        //        trigger_motor_turn_back();
        //				if(++i > TRIGGER_LONG_TIME)  //单次点射拨弹电机持续时间
        //				{
        //						i = 0;
        //						shoot_control.shoot_mode = SHOOT_DONE;
        //				}
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)  // 状态<开始射击-连射>
    {
        // 设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        // 摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        shoot_laser_on();  // 激光开启
        // 计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        // 摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    /* 部分沿用了原有的摩擦轮速度控制逻辑，更改了输出部分，此处只计算电流，主循环里输出电流 */
    //    shoot_fric1_on(shoot_control.fric_pwm1);
    //    shoot_fric2_on(shoot_control.fric_pwm2);
    gimbal_shoot->fric1_motor.speed_set = -shoot_control.fric_pwm1;
    gimbal_shoot->fric2_motor.speed_set = shoot_control.fric_pwm2;
    PID_calc(&gimbal_shoot->fric1_motor_pid, gimbal_shoot->fric1_motor.speed, gimbal_shoot->fric1_motor.speed_set);
    gimbal_shoot->fric1_motor.give_current = (int16_t)(gimbal_shoot->fric1_motor_pid.out);
    PID_calc(&gimbal_shoot->fric2_motor_pid, gimbal_shoot->fric2_motor.speed, gimbal_shoot->fric2_motor.speed_set);
    gimbal_shoot->fric2_motor.give_current = (int16_t)(gimbal_shoot->fric2_motor_pid.out);
    return shoot_control.given_current;
}

/**
 * @brief
 * 射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
    static int16_t last_key_mode = 0;

    // 上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) &&
         shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;  // 状态<摩擦轮开启>
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) &&
              shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;  // 状态<摩擦轮停止>
    }

    if ((shoot_control.shoot_rc->key.v & FRIC_MODE_SWITCH_KEYBOARD) && last_key_mode == 0)
    {
        if (shoot_control.shoot_mode == SHOOT_STOP)
        {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
        else if (shoot_control.shoot_mode != SHOOT_STOP)
        {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
    }
    UI_set_fric(shoot_control.shoot_mode != SHOOT_STOP);
    last_key_mode = (shoot_control.shoot_rc->key.v & FRIC_MODE_SWITCH_KEYBOARD);
    //    //处于中档， 可以使用键盘开启摩擦轮 // 只有通过键盘才能进入这个判断体
    //    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v &
    //    SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    //    {
    //        shoot_control.shoot_mode = SHOOT_READY_FRIC;  // 状态<摩擦轮开启>
    //    }
    //    //处于中档， 可以使用键盘关闭摩擦轮 // 只有通过键盘才能进入这个判断体
    //    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) &&
    //    (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    //    {
    //        shoot_control.shoot_mode = SHOOT_STOP;  // 状态<摩擦轮停止>
    //    }

    // 切换点射和连射
    static int16_t last_mode_key = 0;
    static int8_t shoot_mode = 1;
    if (!last_mode_key && (shoot_control.shoot_rc->key.v & SHOOT_MODE_SWITCH_KEYBOARD))
    {
        shoot_mode = !shoot_mode;
    }
    last_mode_key = shoot_control.shoot_rc->key.v & SHOOT_MODE_SWITCH_KEYBOARD;

    // 两个摩擦轮电机的转速已达到设定值(摩擦轮转速是缓慢增加的)
    if (shoot_control.shoot_mode == SHOOT_READY_FRIC &&
        shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value &&
        shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;  // 状态<开始送弹> (官步功能)
    }
    // 检测送弹是否完成: 微动开关已按下,送弹完成
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
    {
        shoot_control.shoot_mode = SHOOT_READY;  // 状态<可以射击> 送弹完成，准备好射击
    }
    // 检测送弹是否完成: 微动开关未按下,送弹未完成
    else if (shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;  // 状态<开始送弹> (官步功能)
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)  // 状态<可以射击>
    {
        // 下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) ||
            (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;  // 状态<开始射击>
        }
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        /* 官方 */
        //        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        //        {
        //            shoot_control.key_time++;
        //            if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
        //            {
        //                shoot_control.key_time = 0;
        //                shoot_control.shoot_mode = SHOOT_READY_BULLET;
        //            }
        //        }
        //        else
        //        {
        //            shoot_control.key_time = 0;
        //            shoot_control.shoot_mode = SHOOT_BULLET;
        //        }
        /* 新增 */
        shoot_control.key_time++;
        if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }

    // 如果处于射击状态
    if (shoot_control.shoot_mode > SHOOT_READY_FRIC)  // > SHOOT_READY_FRIC: SHOOT_READY_BULLET \ SHOOT_READY
                                                      // \ SHOOT_BULLET \ SHOOT_CONTINUE_BULLET \ SHOOT_DONE
    {
        // 鼠标长按一直进入射击状态 保持连发
        //        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time ==
        //        PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        //        {
        //            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        //        }
        //        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        //        {
        //            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        //        }

        /* 新增：按键切换鼠标点射和连射，遥控器一直连射 */
        if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) ||
            (shoot_mode && shoot_control.press_l))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }

    // 检测枪口热量，若枪口过热，则停止射击
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > 280))
    {
        if (shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }

    // 如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // 拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = { 1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f };

    // 二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] +
                     (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    // 计算输出轴角度
    shoot_control.angle =
        (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    // 微动开关
    //    shoot_control.key = BUTTEN_TRIG_PIN;
    shoot_control.key = SWITCH_TRIGGER_ON;  // 设置微动开关一直处于按下状态，屏蔽送弹检测
    // 鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    // 长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    // 射击开关下档时间计时
    // 新增视觉控制射击的逻辑：即使用视觉指令模拟switch down，从而触发射击
    if (shoot_control.shoot_mode != SHOOT_STOP &&
        (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) ||
         (shoot_control.can_CV->cv_shoot_status == CV_SHOOT_START &&
          shoot_control.can_CV->cv_reco_status == CV_RECO_START)))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    //    static uint16_t up_time = 0;
    //    if (shoot_control.press_r)
    //    {
    //        up_time = UP_ADD_TIME;
    //    }
    //    if (up_time > 0)
    //    {
    //        shoot_control.fric1_ramp.max_value = FRIC_UP;
    //        shoot_control.fric2_ramp.max_value = FRIC_UP;
    //        up_time--;
    //    }
    //    else
    //    {
    //        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
    //        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    //    }
    /* 屏蔽鼠标右键高速射击功能 */
    shoot_control.fric1_ramp.max_value = FRIC_DOWN;
    shoot_control.fric2_ramp.max_value = FRIC_DOWN;
}

static void trigger_motor_turn_back(void)  // 电机旋转
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射 (射出一发子弹)
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
    // 每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_FOUR * 0.95f);  // 由PI_TEN改为PI_FOUR
        shoot_control.move_flag = 1;
    }
    //    if(shoot_control.key == SWITCH_TRIGGER_OFF)  //
    //    微动开关松开：理论上一发弹丸经过会分别按下和松开一次微动开关 (官步逻辑)
    //    {
    //        shoot_control.shoot_mode = SHOOT_DONE;
    //    }
    // 到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)  // 死区=0.05
    {
        // 设定角度没到达，一直旋转
        shoot_control.trigger_speed_set = TRIGGER_SPEED;  // 设置旋转速度
        trigger_motor_turn_back();                        // 旋转
    }
    else
    {
        shoot_control.move_flag = 0;
        /* 新增 */
        shoot_control.shoot_mode = SHOOT_DONE;
    }
}

/**
 * @brief          初始化"gimbal_CV"变量
 * @param[out]     init: 新增了init_CV
 * @retval         none
 */
static void CV_init(shoot_control_t *init)
{
    // 初始化CV结构体
    init->can_CV = get_can_CV_point();
}
