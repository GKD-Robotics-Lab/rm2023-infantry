/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

// PID 模式
typedef enum PID_MODE {
    PID_POSITION = 0, // 位置式
    PID_DELTA         // 增量式
} PID_MODE_t;

// PID 相关参数
typedef struct
{
    PID_MODE_t mode;
    // PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // 最大输出
    fp32 max_iout; // 最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 error[3];  // 误差项 0最新 1上一次 2上上次
    fp32 dead_band; // 误差死区 (绝对值)

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3]; // 微分项 0最新 1上一次 2上上次

    fp32 (*feedforward)(fp32); // 前馈控制的函数指针
    fp32 Fout;

} pid_typedef;

extern void PID_init(pid_typedef *pid, PID_MODE_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 dead_band);
extern void PID_add_feedforward(pid_typedef *pid, fp32 (*feedforward)(fp32));
extern fp32 PID_calc(pid_typedef *pid, fp32 ref, fp32 set);
extern fp32 PID_calc_filterD(pid_typedef *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_typedef *pid);

#endif
