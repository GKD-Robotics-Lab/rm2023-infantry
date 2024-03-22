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
 * >           4.  在"chassis_behaviour_control_set" 函数的最后，添加
 * >               else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
 * >               {
 * >                   chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
 * >               }
 ==============================================================================
 @ endverbatim
 ****************************(C) COPYRIGHT 2019 DJI****************************
 */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"
#include "config.h"

typedef enum {
    CHASSIS_ZERO_FORCE,                  // chassis will be like no power,底盘无力, 跟没上电那样
    CHASSIS_NO_MOVE,                     // chassis will be stop,底盘保持不动
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  // chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
    CHASSIS_NO_FOLLOW_YAW,               // chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                                         // 底盘不跟随角度，角度是开环的，但轮子是有速度环
    CHASSIS_OPEN,                        // the value of remote control will mulitiply a value, get current value that will be sent to can bus
                                         // 遥控器的值乘以比例成电流值 直接发送到can总线上

    //! STEP 1 添加新的行为模式 BEGIN
    CHASSIS_SPIN,                        // 小陀螺模式
    //! STEP 1 添加新的行为模式 END

    CHASSIS_BEHAVIOUR_LEN,               // 表示行为模式的总个数
} chassis_behaviour_e;

//键位状态的结构体
typedef struct
{
    int spin_state;     //陀螺状态
    int last_RC_key;
    /* data */
} chassis_keystate_t;

#define KEY_IN_SPIN 1
#define KEY_OFF_SPIN 0

#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
extern chassis_keystate_t chassis_key_state;

#endif
