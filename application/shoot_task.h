/**
 * @file shoot_task.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//! 射击任务控制时间设置
#define SHOOT_TASK_INIT_TIME 250
#define SHOOT_CONTROL_TIME   1

//! 电机相关参数设置
//* 摩擦轮
#define FRIC_DIRECTION 1 // 摩擦轮旋转方向，0 和 1 代表不同的方向

// 摩擦轮斜坡函数设置
#define FRIC_SPEED_STOP  0.0f
#define FRIC_SPEED_FULL  500.0f // 5000.0f
#define FRIC_SPEED_TURBO 600.0f // 右键加速时的最大速度
#define FRIC_RAMP_ADD    300.0f // 摩擦轮缓启停的加速度
#define FRIC_RAMP_SUB    -200.0f

// 摩擦轮电机 PID
#define FRIC_SPEED_PID_KP        50.0f
#define FRIC_SPEED_PID_KI        0.05f
#define FRIC_SPEED_PID_KD        0.0f
#define FRIC_SPEED_PID_MAX_OUT   10000.0f
#define FRIC_SPEED_PID_MAX_IOUT  10000.0f
#define FRIC_SPEED_PID_DEAD_BAND 0.0f

//* 拨弹轮
// 拨弹轮电机相关参数
#define PIT_NUM                       8  // 拨弹结构一圈坑位数
#define TRIGGER_MOTOR_REDUCTION_RATIO 36 // 拨弹轮电机减速比
#define ECD_RANGE_PER_CIRCLE_RAW 8192
#define ECD_COUNT_PER_CIRCLE (ECD_RANGE_PER_CIRCLE_RAW * TRIGGER_MOTOR_REDUCTION_RATIO)

#define M2006_MOTOR_RPM_TO_VECTOR     (2 * 3.1415926f / (ECD_RANGE * TRIGGER_MOTOR_REDUCTION_RATIO))
#define TRIGGER_MOTOR_ECD_TO_ANGLE    M2006_MOTOR_RPM_TO_VECTOR // 电机编码器反馈值转化为角度 (rad) 的比例

// 拨弹轮速度及堵转相关参数
#define TRIGGER_SPEED               8.0f // 拨弹轮发射时速度
#define TRIGGER_CONTINUE_FIRE_SPEED 15.0f // 拨弹轮连续发射时速度
#define TRIGGER_REVERSE_SPEED       -5.0f // 拨弹轮堵转倒退时速度
#define TRIGGER_BLOCK_SPEED         2.0f  // 认定为堵转时的速度
#define TRIGGER_BLOCK_TIME          700   // 认定为堵转时的时长
#define TRIGGER_REVERSE_TIME        500   // 一次反转的时长

// 拨弹轮电机 PID
#define TRIGGER_ANGLE_PID_KP        1.6f
#define TRIGGER_ANGLE_PID_KI        0.02f
#define TRIGGER_ANGLE_PID_KD        0.0f
#define TRIGGER_ANGLE_PID_MAX_OUT   10.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0.1f
#define TRIGGER_ANGLE_PID_DEAD_BAND 0.0f

#define TRIGGER_SPEED_PID_KP        666.0f
#define TRIGGER_SPEED_PID_KI        0.8f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_SPEED_PID_MAX_OUT   10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT  2000.0f
#define TRIGGER_SPEED_PID_DEAD_BAND 0.0f

//! 遥控器键盘射击状态控制相关参数
#define SHOOT_RC_MODE_CHANNEL   1 // 射击发射开关通道数据
#define SHOOT_ON_KEYBOARD       KEY_PRESSED_OFFSET_G
#define SHOOT_OFF_KEYBOARD      KEY_PRESSED_OFFSET_G

#define PRESS_LONG_TIME         400  // 鼠标长按判断
#define RC_SW_LONG_TIME         1000 // 遥控器射击开关打下档一段时间后，连续发射子弹

#define SHOOT_DONE_KEY_OFF_TIME 15 // 射击完成后 子弹弹出去后，判断时间，以防误触发

//! 枪口热量限制
#define SHOOT_HEAT_REMAIN_VALUE 80

// shoot laser gpio port set 
#define GPIO_Port_laser GPIOC
#define GPIO_PIN_laser GPIO_PIN_8

// 射击模式状态机
typedef enum {
    SHOOT_DISABLE = 0,   // 未使能射击时的无力状态
    SHOOT_START,         // 摩擦轮加速阶段
    SHOOT_READY,         // 摩擦轮已准备好，等待发射
    SHOOT_FIRE,          // 发射一个子弹
    SHOOT_CONTINUE_FIRE, // 连击模式
    SHOOT_DONE,          // 子弹发射完后的阶段
    SHOOT_STOP,          // 失能前的摩擦轮减速阶段
} shoot_mode_e;

// 摩擦轮电机结构体
typedef struct
{
    //* 电机反馈信息
    const motor_measure_t *motor_measure;

    //* 电机 PID 结构体
    pid_typedef motor_pid;

    //* 电机反馈值
    fp32 speed;

    //* 电机设置值
    fp32 *pspeed_set; // 获取 ramp.out 的指针作为电机速度的设置值
    int32_t current_set;
} fric_motor_t;

// 拨弹轮电机结构体
typedef struct
{
    //* 电机反馈信息
    const motor_measure_t *motor_measure;

    //* 电机 PID 结构体
    pid_typedef speed_pid;
    pid_typedef angle_pid;

    //* 电机上一次角度记录
    fp32 last_angle;

    //* 电机反馈值
    uint16_t last_ecd;
    fp32 ecd_count; // 记录电机的电机轴转的圈数，用于将角度归化到输出轴上
    fp32 angle;
    fp32 speed;

    //* 电机设置值
    fp32 speed_set;
    int32_t current_set;

    //* 堵转处理参数
    uint32_t block_time;
    uint32_t reverse_time;
} trigger_motor_t;

// 射击控制数据结构体
typedef struct
{
    //* 射击模式 (状态机)
    shoot_mode_e shoot_mode;
    //* 遥控器指针
    const RC_ctrl_t *shoot_rc;

    //* 射击控制电机结构体
    fric_motor_t fric1;
    fric_motor_t fric2;
    trigger_motor_t trigger;

    //* 摩擦轮斜坡函数
    ramp_function_source_t fric_ramp;

    //* 枪口热量获取
    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

typedef struct
{
    int fric_state;
    int last_fric_state;
    int last_key_state;
} shoot_keyboard_state_t;


void shoot_task(void const *pvParameters);

#endif
