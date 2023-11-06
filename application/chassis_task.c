/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"

#include "bsp_usart.h"

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_strategy(chassis_move_t *chassis_move_strategy);
static void chassis_strategy_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
#ifdef CHASSIS_DEBUG_INPUT_CODE
static fp32 chassis_debug_input(void);
#endif // CHASSIS_DEBUG_INPUT_CODE

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

// 底盘运动数据
chassis_move_t chassis_move;
// 底盘行为模式
extern chassis_behaviour_e chassis_behaviour_mode;

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // wait a time
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // chassis init
    // 底盘初始化
    chassis_init(&chassis_move);

    // make sure all chassis motor is online,
    // 判断底盘电机是否都在线
#ifndef CHASSIS_DEBUG // 在调试时不需要所有电机都在线
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE)) {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
#endif

    while (1) {
        // set chassis control strategy
        // 设置底盘控制策略
        chassis_set_strategy(&chassis_move);
        // when mode changes, some data save
        // 模式切换数据保存
        chassis_strategy_change_control_transit(&chassis_move);
        // chassis data update
        // 底盘数据更新
        chassis_feedback_update(&chassis_move);
        // set chassis control set-point
        // 底盘控制量设置
        chassis_set_contorl(&chassis_move);
        // chassis control pid calculate
        // 底盘控制PID计算
        chassis_control_loop(&chassis_move);

        // make sure  one motor is online at least, so that the control CAN message can be received
        // 确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE))) {
#if defined CHASSIS_DEBUG_INPUT_CODE
            CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                            chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
#else
            // when remote control is offline, chassis motor should receive zero current.
            // 当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE)) {
                CAN_cmd_chassis(0, 0, 0, 0);
            } else {
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
#endif
        }
        // os delay
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化 "chassis_move" 变量，包括 pid 初始化， 遥控器指针初始化，3508 底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init: "chassis_move" 变量指针
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL) {
        return;
    }

    //* 底盘策略初始化
    // 底盘开机状态为原始 (raw)
    chassis_move_init->chassis_translation_strategy      = TRANSLATION_RAW;
    chassis_move_init->chassis_translation_strategy_last = TRANSLATION_RAW;
    chassis_move_init->chassis_rotation_strategy         = ROTATION_DIRECT;
    chassis_move_init->chassis_rotation_strategy_last    = ROTATION_DIRECT;

    //* 底盘整体运动参数初始化
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    // 小陀螺缓启停斜波函数初始化
    ramp_init(&chassis_move_init->chassis_spin_ramp, CHASSIS_CONTROL_TIME, CHASSIS_WZ_SPIN, -CHASSIS_WZ_SPIN);

    //* 底盘电机 PID 结构体初始化
    // get chassis motor data point,  initialize motor speed PID
    // 获取底盘电机数据指针，初始化电机速度环 PID
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    chassis_move_init->motor_chassis[0].chassis_motor_measure = get_motor_measure_point(MOTOR_CHASSIS_1_ID);
    chassis_move_init->motor_chassis[1].chassis_motor_measure = get_motor_measure_point(MOTOR_CHASSIS_2_ID);
    chassis_move_init->motor_chassis[2].chassis_motor_measure = get_motor_measure_point(MOTOR_CHASSIS_3_ID);
    chassis_move_init->motor_chassis[3].chassis_motor_measure = get_motor_measure_point(MOTOR_CHASSIS_4_ID);
    for (uint8_t i = 0; i < 4; i++) {
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_DEAD_BAND);
    }
    // initialize angle PID
    // 初始化角度 PID
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_DEAD_BAND);

    /* 获取底盘角度数据来源指针 */
    // get gyro sensor euler angle point
    // 获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    // get gimbal motor data point
    // 获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor   = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

    //* 遥控器指针获取与滤波初始化
    // get remote control point
    // 获取遥控器指针
    chassis_move_init->chassis_RC               = get_remote_control_point();
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    // first order low-pass filter  replace ramp function
    // 用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    // 键盘控制斜波函数初始化
    ramp_init(&chassis_move_init->key_vx_ramp, CHASSIS_CONTROL_TIME, NORMAL_MAX_CHASSIS_SPEED_X, -NORMAL_MAX_CHASSIS_SPEED_X);
    ramp_init(&chassis_move_init->key_vy_ramp, CHASSIS_CONTROL_TIME, NORMAL_MAX_CHASSIS_SPEED_Y, -NORMAL_MAX_CHASSIS_SPEED_Y);

    //* update data
    // 其他参数会在这里初始化
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          设置底盘移动策略，主要在 'chassis_behaviour_mode_set' 函数中改变
 * @param[out]     chassis_move_mode: "chassis_move" 变量指针.
 * @retval         none
 */
static void chassis_set_strategy(chassis_move_t *chassis_move_strategy)
{
    if (chassis_move_strategy == NULL) {
        return;
    }
    // in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_strategy);
}

/**
 * @brief          底盘模式改变过渡，有些参数需要改变，例如底盘控制 yaw 角度设定值应该变成当前底盘 yaw 角度
 * @param[out]     chassis_move_transit: "chassis_move" 变量指针.
 * @retval         none
 */
static void chassis_strategy_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL) {
        return;
    }

    if (chassis_move_transit->chassis_translation_strategy_last == chassis_move_transit->chassis_translation_strategy &&
        chassis_move_transit->chassis_rotation_strategy_last == chassis_move_transit->chassis_rotation_strategy) {
        return;
    }

    // 切入 ROTATION_DIRECT
    if ((chassis_move_transit->chassis_rotation_strategy_last != ROTATION_DIRECT) && chassis_move_transit->chassis_rotation_strategy == ROTATION_DIRECT) {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // 切入 ROTATION_RELATIVE
    else if ((chassis_move_transit->chassis_rotation_strategy_last != ROTATION_RELATIVE) && chassis_move_transit->chassis_rotation_strategy == ROTATION_RELATIVE) {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    // 切入 ROTATION_ABSOLUTE
    else if ((chassis_move_transit->chassis_rotation_strategy_last != ROTATION_ABSOLUTE) && chassis_move_transit->chassis_rotation_strategy == ROTATION_ABSOLUTE) {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->chassis_translation_strategy_last = chassis_move_transit->chassis_translation_strategy;
    chassis_move_transit->chassis_rotation_strategy_last    = chassis_move_transit->chassis_rotation_strategy;
}

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL) {
        return;
    }

    //* 电机速度更新，数据来自与电机的 CAN 通信
    uint8_t i = 0;
    for (i = 0; i < 4; i++) {
        // update motor speed, accel is differential of speed PID
        // 更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }

    //* 底盘整体运动参数更新，数据根据电机速度计算
    // calculate vertical speed, horizontal speed ,rotation speed, left hand rule
    // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //* 底盘姿态角度更新，数据来自陀螺仪和云台
    // calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    // 计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ANGLE_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ANGLE_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll  = *(chassis_move_update->chassis_INS_angle + INS_ANGLE_ROLL_ADDRESS_OFFSET);
}

/**
 * @brief          设置底盘控制设置值, 三个运动控制值是通过 chassis_behaviour_control_set 函数设置的
 * @param[out]     chassis_move_update: "chassis_move"变量指针.
 * @retval         none
 *
 * @note           该函数计算得到三个底盘整体运动控制值，而不是电机控制值
 * @note           在某些模式中进行了角度的 PID 控制
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL) {
        return;
    }

    //* 根据不同的底盘行为模式，从遥控器获取三个运动控制值
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    // get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //* 根据不同的底盘平动策略，更新 vx_set 和 vy_set
    if (chassis_move_control->chassis_translation_strategy == TRANSLATION_RAW) {
        chassis_move_control->vx_set                      = vx_set;
        chassis_move_control->vy_set                      = vy_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    } else if (chassis_move_control->chassis_translation_strategy == TRANSLATION_VECTOR_FOLLOW_BODY) {
        // speed limit
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    } else if (chassis_move_control->chassis_translation_strategy == TRANSLATION_VECTOR_FOLLOW_GIMBAL) {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        sin_yaw                      = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw                      = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // speed limit
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

    //* 根据不同的底盘转动策略，更新 angle_set
    if (chassis_move_control->chassis_rotation_strategy == ROTATION_DIRECT) {
        // 获得值 angle_set 作为速度直接赋值给 wz_set
        chassis_move_control->wz_set = angle_set;
    } else if (chassis_move_control->chassis_rotation_strategy == ROTATION_RELATIVE) {
        // set control relative angle  set-point
        // 设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
#if defined CHASSIS_DEBUG_INPUT_CODE && defined CHASSIS_DEBUG_ANGEL
        chassis_move_control->chassis_relative_angle_set = chassis_debug_input();
#endif
        // calculate ratation speed
        // 计算旋转PID角速度
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
#if defined CHASSIS_DEBUG_ANGEL && defined PRINT_ON
        usart1_printf("%.2f,%.2f,%.2f\r\n",
                      chassis_move_control->chassis_relative_angle_set,
                      chassis_move_control->wz_set,
                      chassis_move_control->chassis_yaw_motor->relative_angle);
#endif
    } else if (chassis_move_control->chassis_rotation_strategy == ROTATION_ABSOLUTE) {
        // ? TODO 不清楚 我怀疑这个模式和直接控制角度的是一样的，只不过这个转得更慢方便控制
        fp32 delta_angle = 0.0f;
        // set chassis yaw angle set-point
        // 设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delta_angle                           = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        // calculate rotation speed
        // 计算旋转的角速度
        // ? TODO 与上一个模式比较，这里用 delta_angle 是相当于增量式吗，为什么不用 yaw_set 和 yaw 呢？
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delta_angle);
    }
}

/**
 * @brief          four mecanum wheels speed is calculated by three param.
 * @param[in]      vx_set: vertial speed
 * @param[in]      vy_set: horizontal speed
 * @param[in]      wz_set: rotation speed
 * @param[out]     wheel_speed: four mecanum wheels speed
 * @retval         none
 */
/**
 * @brief          将三个运动控制值转化为电机控制值，四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    // TODO 改动
    // because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 max_whell_speed = (chassis_behaviour_mode == CHASSIS_SPIN) ? MAX_WHEEL_SPEED_SPIN : MAX_WHEEL_SPEED;
    fp32 temp            = 0.0f;
    fp32 wheel_speed[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i            = 0;

    //* 将三个运动控制值转化为电机控制值
    // mecanum wheel speed calculation
    // 麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

#if defined CHASSIS_DEBUG_INPUT_CODE && (defined CHASSIS_DEBUG_OPEN || defined CHASSIS_DEBUG_MOTOR_SPEED)
    for (i = 0; i < 4; i++)
        wheel_speed[i] = chassis_debug_input();
#endif // CHASSIS_DEBUG_INPUT_CODE

    //* 原始模式下直接将控制值输出
    if (chassis_move_control_loop->chassis_translation_strategy == TRANSLATION_RAW) {

        for (i = 0; i < 4; i++) {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }

#if defined CHASSIS_DEBUG_OPEN && defined PRINT_ON
        // 这里只输出了电机 1 的参数
        usart1_printf("%d,%.5f\r\n",
                      chassis_move_control_loop->motor_chassis[0].give_current,
                      chassis_move_control_loop->motor_chassis[0].speed);
#endif
        return;
    }

    //* 转速限幅
    // calculate the max speed in four wheels, limit the max speed
    // 计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++) {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp                                                  = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp) {
            max_vector = temp;
        }
    }
    // added 小陀螺模式具有更大的限制速度
    if (max_vector > max_whell_speed) {
        vector_rate = max_whell_speed / max_vector;
        for (i = 0; i < 4; i++) {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //* calculate pid
    for (i = 0; i < 4; i++) {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //* 功率控制
    chassis_power_control(chassis_move_control_loop);

    //* 赋值电流值输出
    for (i = 0; i < 4; i++) {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }

#if defined CHASSIS_DEBUG_MOTOR_SPEED && defined PRINT_ON
    // 这里只输出了电机 1 的参数
    usart1_printf("%.2f,%d,%.2f\r\n",
                  chassis_move_control_loop->motor_chassis[0].speed_set,
                  chassis_move_control_loop->motor_chassis[0].give_current,
                  chassis_move_control_loop->motor_chassis[0].speed);
#endif
}

/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) {
        return;
    }

    static uint32_t last_key_tick = 0;

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;

    // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    // keyboard set speed set-point
    // 键盘控制
    //   加入斜波函数优化操作
    //     x
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY) {
        if (chassis_move_rc_to_vector->key_vx_ramp.out < 0) chassis_move_rc_to_vector->key_vx_ramp.out = 0;
        ramp_calc(&chassis_move_rc_to_vector->key_vx_ramp, RAMP_KEY_ADD_VX);
        vx_set_channel = chassis_move_rc_to_vector->key_vx_ramp.out;
        last_key_tick  = xTaskGetTickCount();
    } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY) {
        if (chassis_move_rc_to_vector->key_vx_ramp.out > 0) chassis_move_rc_to_vector->key_vx_ramp.out = 0;
        ramp_calc(&chassis_move_rc_to_vector->key_vx_ramp, -RAMP_KEY_ADD_VX);
        vx_set_channel = chassis_move_rc_to_vector->key_vx_ramp.out;
        last_key_tick  = xTaskGetTickCount();
    } else if (xTaskGetTickCount() - last_key_tick < 800) // 如果没有按下按键，则清空斜波输出  (用于键盘和遥控器热切换)
    {
        // ramp_to_zerro(&chassis_move_rc_to_vector->key_vx_ramp, RAMP_KEY_ADD_VX * 7);  // 逐渐减小到0，但斜率更大
        chassis_move_rc_to_vector->key_vx_ramp.out = 0;
        vx_set_channel                             = chassis_move_rc_to_vector->key_vx_ramp.out;
    }
    //     y
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY) {
        if (chassis_move_rc_to_vector->key_vy_ramp.out < 0) chassis_move_rc_to_vector->key_vy_ramp.out = 0;
        ramp_calc(&chassis_move_rc_to_vector->key_vy_ramp, RAMP_KEY_ADD_VY);
        vy_set_channel = chassis_move_rc_to_vector->key_vy_ramp.out;
        last_key_tick  = xTaskGetTickCount();
    } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) {
        if (chassis_move_rc_to_vector->key_vy_ramp.out > 0) chassis_move_rc_to_vector->key_vy_ramp.out = 0;
        ramp_calc(&chassis_move_rc_to_vector->key_vy_ramp, -RAMP_KEY_ADD_VY);
        vy_set_channel = chassis_move_rc_to_vector->key_vy_ramp.out;
        last_key_tick  = xTaskGetTickCount();
    } else if (xTaskGetTickCount() - last_key_tick < 800) // 如果没有按下按键，则清空斜波输出  (用于键盘和遥控器热切换)
    {
        // ramp_to_zero(&chassis_move_rc_to_vector->key_vy_ramp, RAMP_KEY_ADD_VY * 7);  // 逐渐减小到0，但斜率更大
        chassis_move_rc_to_vector->key_vy_ramp.out = 0;
        vy_set_channel                             = chassis_move_rc_to_vector->key_vy_ramp.out;
    }

    // first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // stop command, need not slow change, set zero derectly
    // 停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN) {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

#ifdef CHASSIS_DEBUG_INPUT_CODE
/**
 * @brief 作为底盘调试控制值控制输入
 *
 * @return fp32
 */
static fp32 chassis_debug_input(void)
{
    fp32 input      = 0;
    uint32_t period = 3000;                                   // 控制周期 | 单位 ms
    uint32_t t      = (uint32_t)xTaskGetTickCount() % period; // 时间 | 单位 ms | 范围 0~period | tick 约 50 天才会溢出一次，不用担心

    //* 示例 产生方波输入 BEGIN
    if (t < (period / 2)) {
        input = 0;
    } else {
        // input = (fp32)MAX_MOTOR_CAN_CURRENT;
        // input = MAX_WHEEL_SPEED;
        input = PI / 3.0f;
    }
    //* 示例 产生方波输入 END

    return input;
}
#endif // CHASSIS_DEBUG_INPUT_CODE

// TODO 用到的斜波函数的周期统一一下
