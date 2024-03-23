/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
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

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

//! time
#define GIMBAL_TASK_INIT_TIME     201
#define GIMBAL_TASK_WAIT_IMU_TIME 501
#define GIMBAL_CONTROL_TIME       1

//! PID
//*pitch
#define PITCH_SPEED_PID_KP                  5000.0f
#define PITCH_SPEED_PID_KI                  10.0f
#define PITCH_SPEED_PID_KD                  0.0f
#define PITCH_SPEED_PID_MAX_OUT             30000.0f
#define PITCH_SPEED_PID_MAX_IOUT            5000.0f
#define PITCH_SPEED_PID_DEAD_BAND           0.0f

#define PITCH_GYRO_ABSOLUTE_PID_KP          15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI          0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD          0.3f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT     10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT    0.0f
#define PITCH_GYRO_ABSOLUTE_PID_DEAD_BAND   0.0f

#define PITCH_ENCODE_RELATIVE_PID_KP        15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI        0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD        0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f
#define PITCH_ENCODE_RELATIVE_PID_DEAD_BAND 0.0f

//* yaw
#define YAW_SPEED_PID_KP                  7000.0f // 8000.0f // 5390.0f
#define YAW_SPEED_PID_KI                  8.0f // 12.0f   // 51.0f  
#define YAW_SPEED_PID_KD                  0.0f
#define YAW_SPEED_PID_MAX_OUT             30000.0f
#define YAW_SPEED_PID_MAX_IOUT            3000.0f
#define YAW_SPEED_PID_DEAD_BAND           0.0f

#define YAW_GYRO_ABSOLUTE_PID_KP          32.0f
#define YAW_GYRO_ABSOLUTE_PID_KI          0.000f
#define YAW_GYRO_ABSOLUTE_PID_KD          0.2f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT     10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT    0.03f
#define YAW_GYRO_ABSOLUTE_PID_DEAD_BAND   0.0f

#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f
#define YAW_ENCODE_RELATIVE_PID_DEAD_BAND 0.0f

//! others
//* direction
// 云台实际旋转方向是否与电机旋转方向反向
// 1 为正向，-1 为反向
#define PITCH_MOTOR_DIR -1
#define YAW_MOTOR_DIR   1

//* turn
// turn speed
// 掉头云台速度
#define TURN_SPEED 0.04f

//* init
// 云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME   100
#define GIMBAL_INIT_TIME        6000
// 云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
// 初始化时云台要转到的角度
#define INIT_YAW_SET   0.0f
#define INIT_PITCH_SET 0.0f

//* encoder
// 电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE      8191

// 电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f // 2 * PI / 8192
#endif

//* limit
// 云台中值与云台控制值限幅
#define YAW_OFFSET_ECD           3200
#define YAW_MAX_RELATIVE_ANGLE   0.1f
#define YAW_MIN_RELATIVE_ANGLE   -0.1f

#define PITCH_OFFSET_ECD         8000
#define PITCH_MAX_RELATIVE_ANGLE 0.20f
#define PITCH_MIN_RELATIVE_ANGLE -0.25f

//* yaw compensate 
#define YAW_FRIC_COMPENSATION_DEADBAND 0.01f
#define YAW_FRIC_COMPENSATION_GAIN 10000.0f
#define YAW_FRIC_COMPENSATION_MAX 2477.0f
#define YAW_FEEDFORWARD_GAIN 0.001307727781454f

//! remote control
//* channel
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL         2
#define PITCH_CHANNEL       3
#define GIMBAL_MODE_CHANNEL 0
//* deadband
// rocker value deadband
// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10
//* ratio
#define YAW_RC_SEN      -0.000010f
#define PITCH_RC_SEN    -0.000011f // 0.005
#define YAW_MOUSE_SEN   0.00008f
#define PITCH_MOUSE_SEN 0.00014f
//* key define
// turn 180°
// 掉头180 按键
// #define TURN_KEYBOARD KEY_PRESSED_OFFSET_F

typedef enum {
    GIMBAL_MOTOR_RAW = 0,       // 电机原始值控制
    GIMBAL_MOTOR_GYRO,          // 电机陀螺仪角度控制
    GIMBAL_MOTOR_GYRO_LIMIT,    // 带限位的电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE_LIMIT, // 带限位的电机编码值角度控制

    GIMBAL_MOTOR_MODE_LEN,
} gimbal_motor_mode_e;

// 云台电机结构体
typedef struct
{
    //* 电机模式
    gimbal_motor_mode_e motor_mode;
    gimbal_motor_mode_e last_motor_mode;

    //* 电机反馈报文
    const motor_measure_t *motor_measure;

    //* 电机 PID 结构体
    pid_typedef absolute_angle_pid;
    pid_typedef relative_angle_pid;
    pid_typedef speed_pid;

    //* 限位
    uint16_t offset_ecd;     // 电机中值编码
    fp32 max_relative_angle; // rad
    fp32 min_relative_angle; // rad

    //* 角度与角加速度
    fp32 relative_angle;     // rad 与底盘的相对角度
    fp32 relative_angle_set; // rad
    fp32 absolute_angle;     // rad 姿态解算获得的绝对角度
    fp32 absolute_angle_set; // rad
    fp32 speed;              // rad/s
    fp32 speed_set;          // rad/s

    //* 电机设置值
    fp32 raw_cmd_current;
    int16_t current_set;
} gimbal_motor_t;

// 云台控制结构体
typedef struct
{
    //* 云台电机结构体
    gimbal_motor_t yaw_motor;
    gimbal_motor_t pitch_motor;

    //* imu 指针
    const fp32 *INS_angle;
    const fp32 *INS_gyro;

    //* 遥控器指针
    const RC_ctrl_t *rc_ctrl;
} gimbal_control_t;

extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);
extern void gimbal_task(void const *pvParameters);

#endif
