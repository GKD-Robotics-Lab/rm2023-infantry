/**
 * @file config.h
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
     test
*    test
!    test
?    test
>    test
todo test
//   test
*/

//! 蜂鸣器开启宏 BEGIN !//
#define BUZZER_ON
//! 蜂鸣器开启宏 END !//

//! 调试串口开启宏 BEGIN !//
#define PRINT_ON
//! 调试串口开启宏 BEGIN !//

//! 底盘调试宏 BEGIN !//
//* STEP 1 选择要调试的内容 *//
// #define CHASSIS_DEBUG_OPEN // 电机开环测试
// #define CHASSIS_DEBUG_MOTOR_SPEED // 电机速度环调试
// #define CHASSIS_DEBUG_ANGEL // 电机角度环调试
#if defined CHASSIS_DEBUG_OPEN || defined CHASSIS_DEBUG_MOTOR_SPEED || defined CHASSIS_DEBUG_ANGEL
    #define CHASSIS_DEBUG
#endif

#ifdef CHASSIS_DEBUG
    //* STEP 2 选择调试的输入来源 *//
    // #define CHASSIS_DEBUG_INPUT_RC // 控制值来源于遥控器
    #define CHASSIS_DEBUG_INPUT_CODE // 控制值在代码中设置
#endif
//! 底盘调试宏 END !//

//! 云台调试宏 BEGIN !//
//* STEP 1 选择要调试的内容 *//
// #define GIMBAL_DEBUG_OPEN // 电机开环测试
// #define GIMBAL_DEBUG_SPEED // 电机速度环调试
// #define GIMBAL_DEBUG_ABSOLUTE_ANGEL // 电机角度环调试 (使用陀螺仪)
// #define GIMBAL_DEBUG_RELATIVE_ANGEL // 电机角度环调试 (使用编码器)
#if defined GIMBAL_DEBUG_OPEN || defined GIMBAL_DEBUG_SPEED || defined GIMBAL_DEBUG_ABSOLUTE_ANGEL || defined GIMBAL_DEBUG_RELATIVE_ANGEL
    #define GIMBAL_DEBUG
#endif

#ifdef GIMBAL_DEBUG
    //* STEP 2 选择要调试的电机 *//
    // #define GIMBAL_DEBUG_NO_FORCE
    // #define GIMBAL_DEBUG_ONLY_PITCH
    #define GIMBAL_DEBUG_ONLY_YAW
    //* STEP 3 选择调试的输入来源 *//
    // #define GIMBAL_DEBUG_INPUT_RC // 控制值来源于遥控器
    #define GIMBAL_DEBUG_INPUT_CODE // 控制值在代码中设置
#endif
//! 云台调试宏 END !//

//! 校准调试宏 BEGIN !//
//* usart
#ifdef PRINT_ON
    // #define CALI_USART_PRINT // 打开此宏时当校准时会输出调试信息
    #ifdef CALI_USART_PRINT
        #define CALI_USART_GYRO_DATA_PRINT
    #endif
#endif

//* buzzer
#ifdef BUZZER_ON
    // #define CALI_BUZZER_ON // 打开此宏时当校准时会有蜂鸣器提示声

    #ifdef CALI_BUZZER_ON
        #define CALI_GYRO_BUZZER_ON // 设置陀螺仪校准时是否要蜂鸣器提示
    #endif
#endif
//! 校准调试宏 END !//

//! IMU 调试宏 BEGIN !//
#ifdef PRINT_ON
    // #define IMU_DATA_PRINT // 输出欧拉角以及陀螺仪的值以便检查
#endif
//! IMU 调试宏 END !//

#endif