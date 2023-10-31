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

/**
 * @file pid.c
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-02
 *
 * @note PID 计算添加了死区
 * @note 添加了对微分进行滤波的 PID 以抑制高频噪声
 * @note 添加了前馈 feedforward
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "pid.h"
#include "main.h"

#define ABS(x) ((x > 0) ? x : -x)
#define ABS_LIMIT(input, max)      \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

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
void PID_init(pid_typedef *pid, PID_MODE_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 dead_band)
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

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

    pid->feedforward = NULL;
    pid->Fout        = 0.0f;
}

void PID_add_feedforward(pid_typedef *pid, fp32 (*feedforward)(fp32))
{
    pid->feedforward = feedforward;
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
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * pid->error[0];
            pid->Iout += pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * pid->Dbuf[0];
            ABS_LIMIT(pid->Iout, pid->max_iout);
            if (pid->feedforward != NULL) pid->Fout = pid->feedforward(pid->set);
            pid->out = pid->Pout + pid->Iout + pid->Dout + pid->Fout;
            ABS_LIMIT(pid->out, pid->max_out);
        } // 如果在误差死区范围内，沿用上次的输出
    } else if (pid->mode == PID_DELTA) {
        pid->out -= pid->Fout;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * pid->Dbuf[0];
            if (pid->feedforward != NULL) pid->Fout = pid->feedforward(pid->set);
            pid->out += pid->Pout + pid->Iout + pid->Dout + pid->Fout;
            ABS_LIMIT(pid->out, pid->max_out);
        } // 如果在误差死区范围内，沿用上次的输出
    }

    return pid->out;
}

/**
 * @brief          pid 计算 (微分滤波)
 * @param[out]     pid: PID 结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid 输出
 */
fp32 PID_calc_filterD(pid_typedef *pid, fp32 ref, fp32 set)
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
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * pid->error[0];
            pid->Iout += pid->Ki * pid->error[0];
            // 微分滤波
            pid->Dout = pid->Kd * (pid->Dbuf[0] + pid->Dbuf[1] + pid->Dbuf[2]) / 3;
            ABS_LIMIT(pid->Iout, pid->max_iout);
            pid->out = pid->Pout + pid->Iout + pid->Dout;
            if (pid->feedforward != NULL) pid->out += pid->feedforward(pid->set);
            ABS_LIMIT(pid->out, pid->max_out);
        } // 如果在误差死区范围内，沿用上次的输出
    } else if (pid->mode == PID_DELTA) {
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        if (ABS(pid->error[0]) > pid->dead_band) {
            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            // 微分滤波
            pid->Dout = pid->Kd * pid->Kd * (pid->Dbuf[0] + pid->Dbuf[1] + pid->Dbuf[2]) / 3;
            pid->out += pid->Pout + pid->Iout + pid->Dout;
            ABS_LIMIT(pid->out, pid->max_out);
        } // 如果在误差死区范围内，沿用上次的输出
    }

    return pid->out;
}

// TODO 微分先行

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
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
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
