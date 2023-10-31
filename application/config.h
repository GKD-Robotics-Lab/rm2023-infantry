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

//! 底盘调试宏 BEGIN !/
//* STEP 1 选择要调试的内容 */ 
// #define CHASSIS_DEBUG_OPEN // 电机开环测试
// #define CHASSIS_DEBUG_MOTOR_SPEED // 电机速度环调试
// #define CHASSIS_DEBUG_ANGEL // 电机角度环调试

#if defined CHASSIS_DEBUG_OPEN || defined CHASSIS_DEBUG_MOTOR_SPEED || defined CHASSIS_DEBUG_ANGEL
#define CHASSIS_DEBUG
#endif

#ifdef CHASSIS_DEBUG
//* STEP 2 选择调试的输入来源 */
// #define CHASSIS_DEBUG_INPUT_RC // 控制值来源于遥控器
#define CHASSIS_DEBUG_INPUT_CODE // 控制值在代码中设置
#endif
//! 底盘调试宏 END !/

#endif