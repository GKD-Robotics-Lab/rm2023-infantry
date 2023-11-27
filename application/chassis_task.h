/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

/**
 * @file chassis_task.h
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-03
 *
 * @note 代码整理
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "config.h"

//! 底盘参数
//* 底盘控制时间参数
// in the beginning of task ,wait a time
// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
// chassis task control time  2ms
// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// chassis task control time 0.002s
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// chassis control frequence, no use now.
// 底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE (1.0f / CHASSIS_CONTROL_TIME)

//* 底盘计算用参数
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_DISTANCE_TO_CENTER        0.2f
#define CHASSIS_WZ_SET_SCALE            0.0f
// m3508 rmp change to chassis speed,
// m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR       0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
// 小陀螺缓启停的加速度
#define CHASSIS_SPIN_RAMP_ADD 25.0f
#define CHASSIS_SPIN_RAMP_SUB 10.0f
// 小陀螺模式的自旋速度
#define CHASSIS_WZ_SPIN 10.0f
// 小陀螺模式退出的速度
#define CHASSIS_WZ_SPIN_OUT (CHASSIS_WZ_SPIN / 2.0f)

//* 底盘运动限制
// chassis 3508 max motor control current
// 底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
// single chassis motor max speed
// 单个底盘电机最大速度
#define MAX_WHEEL_SPEED      2.6f
#define MAX_WHEEL_SPEED_SPIN 3.0f
// chassis forward or back max speed
// 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
// chassis left or right max speed
// 底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f
// when chassis is not set to move, swing max angle
// 原地不动时底盘摇摆最大角度 (rad)
#define SWING_NO_MOVE_ANGLE 0.7f
// when chassis is set to move, swing max angle
// 运动过程中摇摆底盘最大角度 (rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//* 底盘控制 PID 参数设置
// chassis motor speed PID
// 底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP        15000.0f
#define M3505_MOTOR_SPEED_PID_KI        10.0f
#define M3505_MOTOR_SPEED_PID_KD        0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT   MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT  2000.0f
#define M3505_MOTOR_SPEED_PID_DEAD_BAND 0.0f
// chassis follow angle PID
// 底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP        7.0f // 原始 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI        0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD        0.2f // 原始 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT  0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_DEAD_BAND 0.0f

//! 遥控器参数
//* 遥控器通道设置
// the channel num of controlling vertial speed
// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
// the channel num of controlling horizontal speed
// 左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
// in some mode, can use remote control to control rotation speed
// 在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
// the channel of choosing chassis mode,
// 选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
// press the key, chassis will swing
// 底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
// chassi forward, back, left, right key
// 底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
// 小陀螺模式控制按键
#define CHASSIS_SPIN_KEYBOARD           KEY_PRESSED_OFFSET_R
#define CHASSIS_SPIN_TEMP_STOP_KEYBOARD KEY_PRESSED_OFFSET_SHIFT

//* 遥控器数字转化比例设置
// rocker value (max 660) change to vertial speed (m/s)
// 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
// rocker value (max 660) change to horizontal speed (m/s)
// 遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
// in following yaw angle mode, rocker value add to angle
// 跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
// in not following yaw angle mode, rocker value change to rotation speed
// 不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.009f

//* 遥控器滤波参数
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define RAMP_KEY_ADD_VX     3.0f
#define RAMP_KEY_ADD_VY     3.0f

//* 遥控器摇杆死区设置
// rocker value deadline
// 摇杆死区
#define CHASSIS_RC_DEADLINE 10

typedef enum {
    TRANSLATION_RAW,                  // 原始模式 -> 获得值直接赋给运动参数并发送到 CAN 总线上
    TRANSLATION_VECTOR_FOLLOW_BODY,   // 底盘平动方向跟随车身，不跟随云台
    TRANSLATION_VECTOR_FOLLOW_GIMBAL, // 底盘平动方向跟随云台
} chassis_translation_strategy_e;

typedef enum {
    ROTATION_DIRECT,   // 直接赋值模式 -> 获得值 angle_set 作为速度直接赋值给 wz_set
    ROTATION_RELATIVE, // 旋转角度相对于云台
    ROTATION_ABSOLUTE, // 旋转角度相对于车身
} chassis_rotation_strategy_e;

typedef struct
{
    const motor_measure_t *chassis_motor_measure; // 底盘电机数据指针
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

typedef struct
{
    //* 底盘移动策略
    // 平动策略
    chassis_translation_strategy_e chassis_translation_strategy;
    chassis_translation_strategy_e chassis_translation_strategy_last;
    // 转动策略
    chassis_rotation_strategy_e chassis_rotation_strategy;
    chassis_rotation_strategy_e chassis_rotation_strategy_last;

    //* 底盘整体运动参数
    // 速度实际值
    fp32 vx; // chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy; // chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz; // chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
    // 速度设置值
    fp32 vx_set; // chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set; // chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set; // chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
    // 速度限制值
    fp32 vx_max_speed; // max forward speed, unit m/s.前进方向最大速度 单位m/s
    fp32 vx_min_speed; // max backward speed, unit m/s.后退方向最大速度 单位m/s
    fp32 vy_max_speed; // max letf speed, unit m/s.左方向最大速度 单位m/s
    fp32 vy_min_speed; // max right speed, unit m/s.右方向最大速度 单位m/s
    // 小陀螺缓启停所用的斜波函数
    ramp_function_source_t chassis_spin_ramp;
    // 底盘与云台的相对角度
    const fp32* pchassis_relative_angle;     // the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
    fp32 chassis_relative_angle_set; // the set relative angle.设置相对云台控制角度
    // 底盘的绝对角度，底盘没有陀螺仪时由云台的陀螺仪及电机角度计算得到
    fp32 chassis_yaw; // the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
    fp32 chassis_yaw_set;
    fp32 chassis_pitch; // the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
    fp32 chassis_roll;  // the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度

    //* 底盘电机结构体及其 PID 结构体
    chassis_motor_t motor_chassis[4]; // chassis motor data.底盘电机数据
    pid_typedef motor_speed_pid[4];   // motor speed PID.底盘电机速度pid
    pid_typedef chassis_angle_pid;    // follow angle PID.底盘跟随角度pid

    //* 底盘角度数据来源指针
    const gimbal_motor_t *chassis_yaw_motor;   // will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
    const gimbal_motor_t *chassis_pitch_motor; // will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
    const fp32 *chassis_INS_angle;             // the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针

    //* 底盘遥控器控制及滤波相关参数
    const RC_ctrl_t *chassis_RC;                       // 底盘使用的遥控器指针, the point to remote control
    first_order_filter_type_t chassis_cmd_slow_set_vx; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy; // use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    ramp_function_source_t key_vx_ramp;                // 用于键盘控制的斜波函数
    ramp_function_source_t key_vy_ramp;                // 用于键盘控制的斜波函数

} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
