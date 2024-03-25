/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
  * >           如果要添加一个新的行为模式
  * >           1. 首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
  * >           2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
  * >              "yaw, pitch" 参数是云台运动控制输入量
  * >              第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
  * >              第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
  * >              在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
  * >              最后记得加入到函数数组中
  * >           3. 在"gimbal_behaviour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
  * >              然后选择一种云台控制模式，3种可选:
  * >                  GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
  * >                  GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
  * >                  GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

#include "config.h"
#include "user_lib.h"

#include "auto_aim_task.h"

// 为精简代码将遥控器相关按键使用宏替代
#define RC_gimbal_switch (gimbal_mode_set->rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])
#define RC_mouse_l      (gimbal_mode_set->rc_ctrl->mouse.press_l)               // 鼠标左键
#define RC_mouse_r      (gimbal_mode_set->rc_ctrl->mouse.press_r)               // 鼠标右键

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);

static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_open_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_auto_aim_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void (*gimbal_control_func[GIMBAL_BEHAVIOUR_LEN])(fp32 *, fp32 *, gimbal_control_t *) = {gimbal_zero_force_control,
                                                                                                gimbal_init_control,
                                                                                                gimbal_absolute_angle_control,
                                                                                                gimbal_relative_angle_control,
                                                                                                gimbal_motionless_control,
                                                                                                gimbal_open_control,
                                                                                                gimbal_auto_aim_control};

// 云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL) {
        return;
    }
    // set gimbal_behaviour variable
    // 云台行为状态机设置
    gimbal_behaviour_set(gimbal_mode_set);

    // accoring to gimbal_behaviour, set motor control mode
    // 根据云台行为状态机设置电机状态机
    switch (gimbal_behaviour) {
        case GIMBAL_ZERO_FORCE:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_RAW;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_RAW;
            break;
        case GIMBAL_INIT:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_ENCONDE_LIMIT;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_ENCONDE_LIMIT;
            break;
        case GIMBAL_ABSOLUTE_ANGLE:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_GYRO;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_GYRO_LIMIT;
            break;
        case GIMBAL_RELATIVE_ANGLE:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_ENCONDE_LIMIT;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_ENCONDE_LIMIT;
            break;
        case GIMBAL_MOTIONLESS:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_ENCONDE_LIMIT;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_ENCONDE_LIMIT;
            break;
        case GIMBAL_AUTO_AIM:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_GYRO_DIRECT;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_GYRO_LIMIT;
            break;
        //! STEP 3 为云台行为模式选择对应的电机控制模式 BEGIN !//
        /*
            >   3. 在"gimbal_behaviour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
            >      然后选择一种云台控制模式，3种可选:
            >          GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
            >          GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
            >          GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
        */
        case GIMBAL_OPEN:
            gimbal_mode_set->yaw_motor.motor_mode   = GIMBAL_MOTOR_RAW;
            gimbal_mode_set->pitch_motor.motor_mode = GIMBAL_MOTOR_RAW;
            break;

        //! STEP 3 为云台行为模式选择对应的电机控制模式 END !//
        default:
            break;
    }
}

/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{

    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

    if (gimbal_mode_set == NULL) {
        return;
    }

#ifndef GIMBAL_DEBUG_INPUT_CODE
    //* 遥控器未连接时为无力模式
    // 陀螺仪校准时会关闭遥控器接收，这将导致进入 GIMBAL_ZERO_FORCE，所以不需要判断陀螺仪是否在校准
    if (toe_is_error(DBUS_TOE)) {
        gimbal_behaviour      = GIMBAL_ZERO_FORCE;
        last_gimbal_behaviour = gimbal_behaviour;
        return;
    }
#endif

    //* init 模式下进行是否完成 init 任务的判断
    // init mode, judge if gimbal is in middle place
    // 初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT) {
        static uint16_t init_time      = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if (fabs(gimbal_mode_set->yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
            fabs(gimbal_mode_set->pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
            init_stop_time++;
        if (init_time < GIMBAL_INIT_TIME) init_time++;

        // 超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(RC_gimbal_switch) && !toe_is_error(DBUS_TOE)) {
            return;
        } else {
            init_stop_time = 0;
            init_time      = 0;
        }
    }

    //* 根据拨杆设置遥控器的模式
    // 开关控制 云台状态
    if (switch_is_down(RC_gimbal_switch)) {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    } else if (switch_is_mid(RC_gimbal_switch)) {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    } else if (switch_is_up(RC_gimbal_switch)) {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    //根据拨杆&鼠标右键设置自瞄
    if(((gimbal_mode_set->rc_ctrl->rc.ch[4] > 600) || RC_mouse_r) && AutoAimData.auto_aim_status == AUTOAIM_LOCKED)
    {
        gimbal_behaviour = GIMBAL_AUTO_AIM;
    }

    // //根据拨杆&鼠标右键设置自瞄
    // if((gimbal_mode_set->rc_ctrl->rc.ch[4] > 600) || RC_mouse_r)
    // {
    //     gimbal_behaviour = GIMBAL_AUTO_AIM;
    // }

    //* 在某些模式切换的情况下先进入 init 模式
    // enter init mode
    // 判断进入init状态机
    if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE) {
        gimbal_behaviour = GIMBAL_INIT;
    }

    //* debug 时强制设置对应的模式
#ifdef GIMBAL_DEBUG_OPEN
    gimbal_behaviour      = GIMBAL_OPEN;
#endif

#if defined GIMBAL_DEBUG_SPEED || defined GIMBAL_DEBUG_ABSOLUTE_ANGEL
    gimbal_behaviour      = GIMBAL_ABSOLUTE_ANGLE;
#endif

#ifdef GIMBAL_DEBUG_RELATIVE_ANGEL
    gimbal_behaviour      = GIMBAL_RELATIVE_ANGLE;
#endif

    //* 记录上次的模式
    last_gimbal_behaviour = gimbal_behaviour;

}

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    if (gimbal_control_func[gimbal_behaviour] != NULL) {
        gimbal_control_func[gimbal_behaviour](add_yaw, add_pitch, gimbal_control_set);
    }
}

/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    *yaw   = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 * @author         RM
 * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    //* init 模式控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR) {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw   = 0.0f;
    } else {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw   = (INIT_YAW_SET - gimbal_control_set->yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    //* absolute angle 模式控制量计算
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw   = yaw_channel * YAW_RC_SEN - gimbal_control_set->rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->rc_ctrl->mouse.y * PITCH_MOUSE_SEN;

    // {
    //     //* 掉头控制
    //     static uint16_t last_turn_keyboard = 0;
    //     static uint8_t gimbal_turn_flag    = 0;
    //     static fp32 gimbal_end_angle       = 0.0f;
    //
    //     if ((gimbal_control_set->rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD)) {
    //         if (gimbal_turn_flag == 0) {
    //             gimbal_turn_flag = 1;
    //             // 保存掉头的目标值
    //             gimbal_end_angle = rad_format(gimbal_control_set->yaw_motor.absolute_angle + PI);
    //         }
    //     }
    //     last_turn_keyboard = gimbal_control_set->rc_ctrl->key.v;
    //
    //     if (gimbal_turn_flag) {
    //         // 不断控制到掉头的目标值，正转，反装是随机 (就近)
    //         if (rad_format(gimbal_end_angle - gimbal_control_set->yaw_motor.absolute_angle) > 0.0f) {
    //             *yaw += TURN_SPEED;
    //         } else {
    //             *yaw -= TURN_SPEED;
    //         }
    //     }
    //     // 到达 pi （180°）后停止
    //     if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->yaw_motor.absolute_angle)) < 0.01f) {
    //         gimbal_turn_flag = 0;
    //     }
    // }
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw   = yaw_channel * YAW_RC_SEN - gimbal_control_set->rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }
    *yaw   = 0.0f;
    *pitch = 0.0f;
}

//! STEP 2 实现新的行为控制函数 BEGIN !//
/*
    >           2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
    >              "yaw, pitch" 参数是云台运动控制输入量
    >              第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
    >              第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
    >              在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    >              最后记得加入到函数数组中
*/

/**
 * @brief 云台电机开环控制
 *
 * @param yaw
 * @param pitch
 * @param gimbal_control_set
 */
static void gimbal_open_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(-gimbal_control_set->rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(-gimbal_control_set->rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw   = yaw_channel * 20;
    *pitch = pitch_channel * 20;
}

/*自瞄模式 -Fish*/
static void gimbal_auto_aim_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }else if(AutoAimData.auto_aim_status == AUTOAIM_LOST){
        return;
    }
    //*pitch += (-gimbal_control_set->pitch_motor.absolute_angle - AutoAimData.pitch)*0.0007; //正：下
    *pitch = AutoAimData.pitch; //!会卡死
    *yaw = AutoAimData.yaw;
    //*yaw -= (gimbal_control_set->yaw_motor.absolute_angle - AutoAimData.yaw)*0.009; //正：左

    //!先直接去设陀螺仪目标角度
    gimbal_control_set->yaw_motor.absolute_angle_set = AutoAimData.yaw;

    // usart6_printf("yaw:%f, INS:%f, div:%f\n", AutoAimData.yaw, gimbal_control_set->yaw_motor.absolute_angle,
    //                 AutoAimData.yaw - gimbal_control_set->yaw_motor.absolute_angle);
    // usart6_printf("pitch:%f, INS:%f, div:%f\n", AutoAimData.pitch, gimbal_control_set->pitch_motor.absolute_angle,
    //                 -(gimbal_control_set->pitch_motor.absolute_angle - AutoAimData.pitch));

}

//! STEP 2 实现新的行为控制函数 END !//

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE) {
        return 1;
    } else {
        return 0;
    }
}
