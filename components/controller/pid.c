/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "user_lib.h"

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID 结构数据指针
 * @param[in]      mode: PID_POSITION: 位置式 PID
 *                       PID_DELTA: 增量式 PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid 最大输出
 * @param[in]      max_iout: pid 最大积分输出
 * @param[in]      dead_band: pid 误差死区
 * @retval         none
 */
void PID_init(pid_typedef *pid, pid_mode_e mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 dead_band)
{
    if (pid == NULL || PID == NULL) {
        return;
    }
    pid->mode = mode;

    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];

    pid->max_out   = max_out;
    pid->max_iout  = max_iout;
    pid->dead_band = dead_band;

    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->last_diff                                                                               = 0.0f;

    pid->compensate = NULL;
    pid->Cout       = 0.0f;
}

/**
 * @brief 为 pid 添加补偿
 *
 * @param pid
 * @param compensate 补偿处理函数的函数指针
 */
void PID_add_compensate(pid_typedef *pid, fp32 (*compensate)(pid_typedef *))
{
    pid->compensate = compensate;
}

/**
 * @brief          pid 计算 (普通)
 * @param[out]     pid: PID 结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid 输出
 */
fp32 PID_calc(pid_typedef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    pid->set = set;
    pid->fdb = ref;

    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION) {
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * pid->error[0];
            pid->Iout += pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);
            ABS_LIMIT(pid->Iout, pid->max_iout);
            pid->out = pid->Pout + pid->Iout + pid->Dout;
            ABS_LIMIT(pid->out, pid->max_out);
            if (pid->compensate != NULL) {
                pid->Cout = pid->compensate(pid);
                pid->out += pid->Cout;
                ABS_LIMIT(pid->out, pid->max_out);
            }
        } // 如果在误差死区范围内，沿用上次的输出
    } else if (pid->mode == PID_DELTA) {
        pid->out -= pid->Cout;
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
            pid->out += pid->Pout + pid->Iout + pid->Dout;
            ABS_LIMIT(pid->out, pid->max_out);
            if (pid->compensate != NULL) {
                pid->Cout = pid->compensate(pid);
                pid->out += pid->Cout;
                ABS_LIMIT(pid->out, pid->max_out);
            }
        } // 如果在误差死区范围内，沿用上次的输出
    }

    return pid->out;
}

/**
 * @brief          pid 计算 (指定微分)
 * @note           适用于例如云台角度环将陀螺仪输出指定为微分项 D，也可以用于自己对微分项 D 进行滤波
 * @param[out]     pid: PID 结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @param[in]      diff: 指定的微分值
 * @retval         pid 输出
 */
fp32 PID_calc_specifyD(pid_typedef *pid, fp32 ref, fp32 set, fp32 diff)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    pid->set = set;
    pid->fdb = ref;

    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION) {
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * pid->error[0];
            pid->Iout += pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * diff;
            ABS_LIMIT(pid->Iout, pid->max_iout);
            pid->out = pid->Pout + pid->Iout + pid->Dout;
            ABS_LIMIT(pid->out, pid->max_out);
            if (pid->compensate != NULL) {
                pid->Cout = pid->compensate(pid);
                pid->out += pid->Cout;
                ABS_LIMIT(pid->out, pid->max_out);
            }
        } // 如果在误差死区范围内，沿用上次的输出
    } else if (pid->mode == PID_DELTA) {
        pid->out -= pid->Cout;
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * (diff - pid->last_diff);
            pid->out += pid->Pout + pid->Iout + pid->Dout;
            ABS_LIMIT(pid->out, pid->max_out);
            if (pid->compensate != NULL) {
                pid->Cout = pid->compensate(pid);
                pid->out += pid->Cout;
                ABS_LIMIT(pid->out, pid->max_out);
            }
            pid->last_diff = diff;
        } // 如果在误差死区范围内，沿用上次的输出
    }

    return pid->out;
}

// TODO 微分先行

/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID 结构数据指针
 * @retval         none
 */
void PID_clear(pid_typedef *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->last_diff                                = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
