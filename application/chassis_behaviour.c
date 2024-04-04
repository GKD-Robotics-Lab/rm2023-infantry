/**
 ****************************(C) COPYRIGHT 2019 DJI****************************
 * @file       chassis_behaviour.c/h
 * @brief      according to remote control, change the chassis behaviour.
 *             根据遥控器的值，决定底盘行为。
 * @note
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Dec-26-2018     RM              1. done
 *  V1.1.0     Nov-11-2019     RM              1. add some annotation
 *
 @ verbatim
 ==============================================================================
 * >           如果要添加一个新的行为模式
 * >           1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
 * >           erum
 * >           {
 * >               ...
 * >               ...
 * >               CHASSIS_XXX_XXX, // 新添加的
 * >           }chassis_behaviour_e,
 * >           2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
 * >               "vx,vy,wz" 参数是底盘运动控制输入量
 * >               第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
 * >               第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
 * >               第三个参数: 'wz' 可能是角度控制或者旋转速度控制
 * >               在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
 * >           3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
 * >               在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择底盘控制策略
 * >           4.  在 chassis_behaviour_control_func 数组里添加对应的函数
 ==============================================================================
 @ endverbatim
 ****************************(C) COPYRIGHT 2019 DJI****************************
 */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "detect_task.h"

#include "custom_ui_task.h"

#define RC_chassis_switch (behaviour_set->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) // 遥控器底盘控制拨杆

static void chassis_behaviour_set(chassis_move_t *behaviour_set);

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void (*chassis_behaviour_control_func[CHASSIS_BEHAVIOUR_LEN])(fp32 *, fp32 *, fp32 *, chassis_move_t *) = {chassis_zero_force_control,
                                                                                                                  chassis_no_move_control,
                                                                                                                  chassis_infantry_follow_gimbal_yaw_control,
                                                                                                                  chassis_no_follow_yaw_control,
                                                                                                                  chassis_open_set_control,
                                                                                                                  chassis_spin_control};

// 单独放在这防止了文件互相包含的问题，也方便了添加新模式
// highlight, the variable chassis behaviour mode
// 底盘行为模式
chassis_behaviour_e chassis_behaviour_mode             = CHASSIS_ZERO_FORCE;
static chassis_behaviour_e chassis_behaviour_mode_last = CHASSIS_ZERO_FORCE;
chassis_keystate_t chassis_key_state;

/**
 * @brief          通过遥控器获取底盘行为模式，通过逻辑判断选择底盘控制模式 chassis_behaviour_mode
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL) {
        return;
    }

    chassis_behaviour_set(chassis_move_mode);

    //* 更新UI状态
    if (chassis_behaviour_mode == CHASSIS_SPIN){
        UI_Data.spin_state = 1;
    }else{
        UI_Data.spin_state = 0;
    }

    //chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

    //* 根据行为模式选择底盘移动策略
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_RAW;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_DIRECT;
    } else if (chassis_behaviour_mode == CHASSIS_NO_MOVE) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_VECTOR_FOLLOW_BODY;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_DIRECT;
    } else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_VECTOR_FOLLOW_GIMBAL;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_RELATIVE;
    } else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_VECTOR_FOLLOW_BODY;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_DIRECT;
    } else if (chassis_behaviour_mode == CHASSIS_OPEN) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_RAW;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_DIRECT;
    }
    //! STEP 3 添加新的行为模式对应的策略 BEGIN
    else if (chassis_behaviour_mode == CHASSIS_SPIN) {
        chassis_move_mode->chassis_translation_strategy = TRANSLATION_VECTOR_FOLLOW_GIMBAL;
        chassis_move_mode->chassis_rotation_strategy    = ROTATION_DIRECT;
    }

    //! STEP 3 添加新的行为模式对应的控制模式 END
}


static void chassis_behaviour_set(chassis_move_t *behaviour_set)
{
    //*根据键位开关模式
    if((behaviour_set->chassis_RC->key.v & CHASSIS_SPIN_TOGGLE_KEYBOARD && !(chassis_key_state.last_RC_key & CHASSIS_SPIN_TOGGLE_KEYBOARD))){
        if(chassis_key_state.spin_state == KEY_IN_SPIN){
            chassis_key_state.spin_state = KEY_OFF_SPIN;
        }else if(chassis_key_state.spin_state == KEY_OFF_SPIN){
            chassis_key_state.spin_state = KEY_IN_SPIN;
        }
    }
    chassis_key_state.last_RC_key = behaviour_set->chassis_RC->key.v;


    //*软件Reset
    if(behaviour_set->chassis_RC->key.v & SOFT_RESET_KEY){
         HAL_NVIC_SystemReset();
    }


    //* 遥控器设置底盘行为模式
    if (switch_is_down(RC_chassis_switch)) {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }else if(behaviour_set->chassis_RC->key.v & CHASSIS_SPIN_TEMP_STOP_KEYBOARD){
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;                //暂停小陀螺
    }else if(chassis_key_state.spin_state == KEY_IN_SPIN){
        chassis_behaviour_mode = CHASSIS_SPIN;           
    }else if (switch_is_mid(RC_chassis_switch)) {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;                //遥控器控制陀螺（低优先级
    } else if (switch_is_up(RC_chassis_switch)) {
        chassis_behaviour_mode = CHASSIS_SPIN;
    }

    //* 进出小陀螺模式的过渡
    /**if (chassis_behaviour_mode_last != CHASSIS_SPIN && chassis_behaviour_mode == CHASSIS_SPIN) {*/
    /**    behaviour_set->chassis_spin_ramp.out = behaviour_set->wz;*/
    /**    // 设置小陀螺启动增速*/
    /**    if (behaviour_set->wz >= 0)*/
    /**        behaviour_set->chassis_spin_ramp.input = CHASSIS_SPIN_RAMP_ADD;*/
    /**    else*/
    /**        behaviour_set->chassis_spin_ramp.input = -CHASSIS_SPIN_RAMP_ADD;*/
    /**} else if (chassis_behaviour_mode_last == CHASSIS_SPIN && chassis_behaviour_mode != CHASSIS_SPIN) {*/
    /**    // 若小陀螺速度已减至足够小并且在不会反转的区间，则退出小陀螺模式*/
    /**    if (ABS(behaviour_set->chassis_spin_ramp.out) > CHASSIS_WZ_SPIN_OUT) {*/
    /**        // 设置小陀螺退出减速*/
    /**        if (behaviour_set->wz >= 0)*/
    /**            behaviour_set->chassis_spin_ramp.input = -CHASSIS_SPIN_RAMP_SUB;*/
    /**        else*/
    /**            behaviour_set->chassis_spin_ramp.input = CHASSIS_SPIN_RAMP_SUB;*/
    /**        chassis_behaviour_mode = CHASSIS_SPIN; // 继续在小陀螺模式中减速*/
    /**    } else {*/
    /**        behaviour_set->chassis_spin_ramp.input = 0;*/
    /**        // BUG 还没退出小陀螺模式又将拨杆拨上会进入小陀螺低速旋转模式*/
    /**        [>if (((int)behaviour_set->wz ^ (int)(*behaviour_set->pchassis_relative_angle)) > 0)<]*/
    /**            behaviour_set->chassis_spin_ramp.out = 0; // 清空输出*/
    /**        [>else<]*/
    /**            [>chassis_behaviour_mode = CHASSIS_SPIN; // 否则继续在小陀螺模式中减速<]*/
    /**    }*/
    /**}*/

#ifndef CHASSIS_DEBUG
    //* 检查是否需要强制底盘不动
    // when gimbal in some mode, such as init mode, chassis must's move
    // 当云台在某些模式下，像初始化， 底盘不动
		if (gimbal_cmd_to_chassis_stop()) {
				chassis_behaviour_mode = CHASSIS_NO_MOVE;
		}
#endif

#ifndef CHASSIS_DEBUG_INPUT_CODE
    //* 遥控器未连接时为无力模式
    if (toe_is_error(DBUS_TOE)) {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
#endif

    //* 记录底盘行为模式
    chassis_behaviour_mode_last = chassis_behaviour_mode;

    //* 调试时强制设置行为模式
#if defined CHASSIS_DEBUG_OPEN
    chassis_behaviour_mode = CHASSIS_OPEN;
#elif defined CHASSIS_DEBUG_MOTOR_SPEED
    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
#elif defined CHASSIS_DEBUG_ANGEL
    chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
#endif
}

/**
 * @brief          控制值获取：对于不同的底盘行为模式，根据遥控器的选择获取三个运动控制值. 在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set 通常控制纵向移动.
 * @param[out]     vy_set 通常控制横向移动.
 * @param[out]     wz_set 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector 包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }

    if (chassis_behaviour_control_func[chassis_behaviour_mode] != NULL)
        chassis_behaviour_control_func[chassis_behaviour_mode](vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 * @retval         返回空
 */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 * @retval         返回空
 */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @param[out]     vx_set 前进的速度,正值 前进速度， 负值 后退速度
 * @param[out]     vy_set 左右的速度,正值 左移速度， 负值 右移速度
 * @param[out]     angle_set 底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 *
 * @note 可以设置底盘摇摆
 *
 * @retval         返回空
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }

    // channel value and keyboard value change to speed set-point, in general
    // 根据遥控器的通道值以及键盘按键，得出一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //* 底盘摇摆相关参数初始化
    // 底盘摇摆的角度跟随一个 sin 函数，摇摆的幅值为 max_angle
    // swing angle is generated by sin function, swing_time is the input time of sin
    // 摇摆角度是利用 sin 函数生成，swing_time 是 sin 函数的输入值
    static fp32 swing_time  = 0.0f;
    static fp32 swing_angle = 0.0f;
    // max_angle is the max angle that chassis will ratate
    // max_angle 是 sin 函数的幅值
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    // swing_time  plus the add_time in one control cycle
    // swing_time 在一个控制周期内，加上 add_time
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    static uint8_t swing_flag  = 0;

    //* 判断是否要摇摆
    // judge if swing
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY) {
        if (swing_flag == 0) {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    } else {
        swing_flag = 0;
    }

    //* 在底盘运动时减小摇摆角度
    // judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    // 判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) {
        max_angle = SWING_MOVE_ANGLE;
    } else {
        max_angle = SWING_NO_MOVE_ANGLE;
    }

    //* 摇摆角度计算及参数处理
    if (swing_flag) {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    } else {
        swing_angle = 0.0f;
    }
    // swing_time  range [0, 2*PI]
    // sin函数不超过2pi
    if (swing_time > 2 * PI) {
        swing_time -= 2 * PI;
    }

    //* 改模式下摇摆的角度就是底盘的相对角度
    *angle_set = swing_angle;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set 前进的速度, 正值 前进速度， 负值 后退速度
 * @param[in]      vy_set 左右的速度, 正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 底盘设置的旋转速度, 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set 前进的速度，正值 前进速度， 负值 后退速度
 * @param[in]      vy_set 左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 * @retval         none
 */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

//! STEP 2 实现新的行为模式控制函数 BEGIN
/*
>   实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
>       "vx,vy,wz" 参数是底盘运动控制输入量
>       第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
>       第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
>       第三个参数: 'wz' 可能是角度控制或者旋转速度控制
>       在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
>   记得添加函数声明
*/

/**
 * @brief          小陀螺模式，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[out]     vx_set 前进的速度,正值 前进速度， 负值 后退速度
 * @param[out]     vy_set 左右的速度,正值 左移速度， 负值 右移速度
 * @param[out]     wz_set 底盘设置的旋转速度，正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector 底盘数据
 * @retval         返回空
 */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }

    // 可以实现摇摆叠加
    // static fp32 ts = 0.0;
    // ts += 5e-3;

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //* 使用斜波函数为底盘增加角速度
    ramp_calc(&chassis_move_rc_to_vector->chassis_spin_ramp, chassis_move_rc_to_vector->chassis_spin_ramp.input);
    *wz_set = CHASSIS_WZ_SPIN/* * (0.75 - 0.25 * sin(ts))*/;
}

//! STEP 2 实现新的行为模式控制函数 END
