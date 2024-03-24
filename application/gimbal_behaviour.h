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

#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "struct_typedef.h"
#include "gimbal_task.h"

// 云台行为模式
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 无力模式
    GIMBAL_INIT, // 云台转到中值位置 -> 云台刚上电、无力模式退出时进入，超时、开关打下档、遥控器掉线时退出
    GIMBAL_ABSOLUTE_ANGLE, // 根据陀螺仪进行绝对角度控制
    GIMBAL_RELATIVE_ANGLE, // 根据编码器进行相对底盘的相对角度控制
    GIMBAL_MOTIONLESS, // 不动模式，相对底盘电机各轴增量为 0
    //! STEP 1 添加新的行为模式名称 BEGIN !//
    GIMBAL_OPEN, // 开环模式，用于调试
    GIMBAL_AUTO_AIM, //自瞄模式

    //! STEP 1 添加新的行为模式名称 END !//
    GIMBAL_BEHAVIOUR_LEN,
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern bool_t gimbal_cmd_to_shoot_stop(void);

#endif
