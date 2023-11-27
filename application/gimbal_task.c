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

#include "gimbal_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "config.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "pid.h"

#ifdef GIMBAL_DEBUG
#include "bsp_usart.h"
#include "string.h"
extern UART_HandleTypeDef huart1;
#endif

// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

static void gimbal_init(void);
static void gimbal_set_mode(void);
static void gimbal_feedback_update(void);
static void gimbal_mode_change_control_transit(void);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_set_control(void);
static void gimbal_control_loop(void);

static void gimbal_motor_raw_set_control(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_motor_gyro_set_control(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_motor_gyro_limit_set_control(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_motor_encoder_limit_set_control(gimbal_motor_t *gimbal_motor, fp32 add);
static void (*gimbal_motor_set_control_func[GIMBAL_MOTOR_MODE_LEN])(gimbal_motor_t *, fp32) = {gimbal_motor_raw_set_control,
                                                                                               gimbal_motor_gyro_set_control,
                                                                                               gimbal_motor_gyro_limit_set_control,
                                                                                               gimbal_motor_encoder_limit_set_control};

static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
static void (*gimbal_motor_control_func[GIMBAL_MOTOR_MODE_LEN])(gimbal_motor_t *) = {gimbal_motor_raw_angle_control,
                                                                                     gimbal_motor_absolute_angle_control,
                                                                                     gimbal_motor_absolute_angle_control,
                                                                                     gimbal_motor_relative_angle_control};

static fp32 gimbal_motor_yaw_speed_compensate(pid_typedef *pid);

#ifdef GIMBAL_DEBUG_INPUT_CODE
static fp32 gimbal_debug_input(void);
#endif

// gimbal control data
// 云台控制所有相关数据
gimbal_control_t gimbal_control;

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    // wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // gimbal init
    // 云台初始化
    gimbal_init();
    // wait for all motor online
    // 判断电机是否都上线并且 IMU 开始工作
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE) || toe_is_error(BOARD_ACCEL_TOE) || toe_is_error(BOARD_GYRO_TOE)) {
        vTaskDelay(GIMBAL_CONTROL_TIME);
        gimbal_feedback_update(); // 云台数据反馈
    }
    // 延时一段时间等待 IMU 的解算值稳定下来
    vTaskDelay(GIMBAL_TASK_WAIT_IMU_TIME);

    while (1) {
        gimbal_set_mode();                    // 设置云台控制模式
        gimbal_mode_change_control_transit(); // 控制模式切换 控制数据过渡
        gimbal_feedback_update();             // 云台数据反馈
        gimbal_set_control();                 // 设置云台控制量
        gimbal_control_loop();                // 云台控制PID计算

#if defined GIMBAL_DEBUG_OPEN && defined PRINT_ON
        {
            static uint8_t data[8] = {0};
            volatile fp32 current  = (fp32)gimbal_control.yaw_motor.current_set;
            memcpy(&data[0], &gimbal_control.yaw_motor.speed, 4);
            memcpy(&data[4], &current, 4);
            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
                HAL_UART_Transmit_DMA(&huart1, data, 8);
        }
        // usart1_printf("%f, %f, %f, %d\r\n", gimbal_control.yaw_motor.absolute_angle, gimbal_control.yaw_motor.relative_angle, gimbal_control.yaw_motor.speed, gimbal_control.yaw_motor.current_set);
        // usart1_printf("%f, %f, %f, %d\r\n", gimbal_control.pitch_motor.absolute_angle, gimbal_control.pitch_motor.relative_angle, gimbal_control.pitch_motor.speed, gimbal_control.pitch_motor.current_set);
#endif

#if defined GIMBAL_DEBUG_SPEED && defined PRINT_ON
        {
            static uint8_t data[8] = {0};
            memcpy(&data[0], &gimbal_control.yaw_motor.speed, 4);
            memcpy(&data[4], &gimbal_control.yaw_motor.speed_set, 4);
            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
                HAL_UART_Transmit_DMA(&huart1, data, 8);
        }
        // usart1_printf("%f, %f, %d\r\n", gimbal_control.yaw_motor.speed, gimbal_control.yaw_motor.speed_set, gimbal_control.yaw_motor.current_set);
        // usart1_printf("%f, %f, %d\r\n", gimbal_control.pitch_motor.speed, gimbal_control.pitch_motor.speed_set, gimbal_control.pitch_motor.current_set);
#endif

#if defined GIMBAL_DEBUG_ABSOLUTE_ANGEL && defined PRINT_ON
        {
            static uint8_t data[8] = {0};
            memcpy(&data[0], &gimbal_control.pitch_motor.absolute_angle, 4);
            memcpy(&data[4], &gimbal_control.pitch_motor.absolute_angle_set, 4);
            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
                HAL_UART_Transmit_DMA(&huart1, data, 8);
        }
        // usart1_printf("%f, %f, %f, %f, %f, %f, %d\r\n", gimbal_control.yaw_motor.absolute_angle, gimbal_control.yaw_motor.absolute_angle_set, gimbal_control.yaw_motor.relative_angle, gimbal_control.yaw_motor.relative_angle_set, gimbal_control.yaw_motor.speed, gimbal_control.yaw_motor.speed_set, gimbal_control.yaw_motor.current_set);
        // usart1_printf("%.3f,%.3f,%.3f\r\n", gimbal_control.yaw_motor.absolute_angle, gimbal_control.yaw_motor.absolute_angle_set, gimbal_control.yaw_motor.relative_angle);
#endif

        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE))) {
            if (toe_is_error(DBUS_TOE)) {
                CAN_cmd_gimbal(0, 0, 0, 0);
            } else {
#if defined GIMBAL_DEBUG_NO_FORCE
                CAN_cmd_gimbal(0, 0, 0, 0);
#elif defined GIMBAL_DEBUG_ONLY_YAW
                CAN_cmd_gimbal(gimbal_control.yaw_motor.current_set, 0, 0, 0);
#elif defined GIMBAL_DEBUG_ONLY_PITCH
                CAN_cmd_gimbal(0, gimbal_control.pitch_motor.current_set, 0, 0);
#else
                CAN_cmd_gimbal(gimbal_control.yaw_motor.current_set, gimbal_control.pitch_motor.current_set, 0, 0);
#endif
            }
        }

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 *
 * @retval         none
 */
static void gimbal_init(void)
{
    static const fp32 Yaw_absolute_angle_pid[3]   = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_relative_angle_pid[3]   = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
    static const fp32 Yaw_speed_pid[3]            = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    static const fp32 Pitch_absolute_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Pitch_relative_angle_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    static const fp32 Pitch_speed_pid[3]          = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};

    //* 电机反馈报文指针获取
    gimbal_control.yaw_motor.motor_measure   = get_motor_measure_point(MOTOR_GIMBAL_YAW_ID);
    gimbal_control.pitch_motor.motor_measure = get_motor_measure_point(MOTOR_GIMBAL_PITCH_ID);

    //* 姿态数据指针获取
    gimbal_control.INS_angle = get_INS_angle_point();
    gimbal_control.INS_gyro  = get_gyro_data_point();

    //* 遥控器控制指针获取
    gimbal_control.rc_ctrl = get_remote_control_point();

    //* 初始化电机模式
    gimbal_control.yaw_motor.motor_mode = gimbal_control.yaw_motor.last_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_control.pitch_motor.motor_mode = gimbal_control.pitch_motor.last_motor_mode = GIMBAL_MOTOR_RAW;

    //* 设置云台中值与云台控制值限幅
    gimbal_control.yaw_motor.offset_ecd         = YAW_OFFSET_ECD;
    gimbal_control.yaw_motor.max_relative_angle = YAW_MAX_RELATIVE_ANGLE;
    gimbal_control.yaw_motor.min_relative_angle = YAW_MIN_RELATIVE_ANGLE;

    gimbal_control.pitch_motor.offset_ecd         = PITCH_OFFSET_ECD;
    gimbal_control.pitch_motor.max_relative_angle = PITCH_MAX_RELATIVE_ANGLE;
    gimbal_control.pitch_motor.min_relative_angle = PITCH_MIN_RELATIVE_ANGLE;

    //* 初始化电机 PID
    // yaw
    PID_init(&gimbal_control.yaw_motor.absolute_angle_pid, PID_POSITION, Yaw_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_DEAD_BAND);
    PID_init(&gimbal_control.yaw_motor.relative_angle_pid, PID_POSITION, Yaw_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_DEAD_BAND);
    PID_init(&gimbal_control.yaw_motor.speed_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT, YAW_SPEED_PID_DEAD_BAND);
    // pitch
    PID_init(&gimbal_control.pitch_motor.absolute_angle_pid, PID_POSITION, Pitch_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_DEAD_BAND);
    PID_init(&gimbal_control.pitch_motor.relative_angle_pid, PID_POSITION, Pitch_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_DEAD_BAND);
    PID_init(&gimbal_control.pitch_motor.speed_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT, PITCH_SPEED_PID_DEAD_BAND);

    PID_add_compensate(&gimbal_control.yaw_motor.speed_pid, gimbal_motor_yaw_speed_compensate);

    //* 进行一次反馈数据更新并设置初始值
    gimbal_feedback_update();

    gimbal_control.yaw_motor.absolute_angle_set = gimbal_control.yaw_motor.absolute_angle;
    gimbal_control.yaw_motor.relative_angle_set = gimbal_control.yaw_motor.relative_angle;
    gimbal_control.yaw_motor.speed_set          = gimbal_control.yaw_motor.speed;

    gimbal_control.pitch_motor.absolute_angle_set = gimbal_control.pitch_motor.absolute_angle;
    gimbal_control.pitch_motor.relative_angle_set = gimbal_control.pitch_motor.relative_angle;
    gimbal_control.pitch_motor.speed_set          = gimbal_control.pitch_motor.speed;
}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 *
 * @retval         none
 */
static void gimbal_set_mode(void)
{
    gimbal_behaviour_mode_set(&gimbal_control);
}

/**
 * @brief          云台数据更新，包括电机速度，欧拉角度，机器人速度
 *
 * @retval         none
 */
static void gimbal_feedback_update(void)
{
    //* pitch
    gimbal_control.pitch_motor.absolute_angle = *(gimbal_control.INS_angle + INS_ANGLE_PITCH_ADDRESS_OFFSET);

    gimbal_control.pitch_motor.relative_angle = PITCH_MOTOR_DIR * motor_ecd_to_angle_change(gimbal_control.pitch_motor.motor_measure->ecd, gimbal_control.pitch_motor.offset_ecd);

    gimbal_control.pitch_motor.speed = *(gimbal_control.INS_gyro + INS_GYRO_Y_ADDRESS_OFFSET);

    //* yaw
    gimbal_control.yaw_motor.absolute_angle = *(gimbal_control.INS_angle + INS_ANGLE_YAW_ADDRESS_OFFSET);

    gimbal_control.yaw_motor.relative_angle = YAW_MOTOR_DIR * motor_ecd_to_angle_change(gimbal_control.yaw_motor.motor_measure->ecd, gimbal_control.yaw_motor.offset_ecd);

    gimbal_control.yaw_motor.speed = arm_cos_f32(gimbal_control.pitch_motor.relative_angle) * (*(gimbal_control.INS_gyro + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(gimbal_control.pitch_motor.relative_angle) * (*(gimbal_control.INS_gyro + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    } else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 *
 * @retval         none
 */
static void gimbal_mode_change_control_transit(void)
{
    // yaw电机状态机切换保存数据
    if (gimbal_control.yaw_motor.last_motor_mode != GIMBAL_MOTOR_RAW && gimbal_control.yaw_motor.motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_control.yaw_motor.raw_cmd_current = gimbal_control.yaw_motor.current_set = gimbal_control.yaw_motor.current_set;
    } else if (gimbal_control.yaw_motor.last_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_control.yaw_motor.motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_control.yaw_motor.absolute_angle_set = gimbal_control.yaw_motor.absolute_angle;
    } else if (gimbal_control.yaw_motor.last_motor_mode != GIMBAL_MOTOR_GYRO_LIMIT && gimbal_control.yaw_motor.motor_mode == GIMBAL_MOTOR_GYRO_LIMIT) {
        gimbal_control.yaw_motor.absolute_angle_set = gimbal_control.yaw_motor.absolute_angle;
    } else if (gimbal_control.yaw_motor.last_motor_mode != GIMBAL_MOTOR_ENCONDE_LIMIT && gimbal_control.yaw_motor.motor_mode == GIMBAL_MOTOR_ENCONDE_LIMIT) {
        gimbal_control.yaw_motor.relative_angle_set = gimbal_control.yaw_motor.relative_angle;
    }
    gimbal_control.yaw_motor.last_motor_mode = gimbal_control.yaw_motor.motor_mode;

    // pitch电机状态机切换保存数据
    if (gimbal_control.pitch_motor.last_motor_mode != GIMBAL_MOTOR_RAW && gimbal_control.pitch_motor.motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_control.pitch_motor.raw_cmd_current = gimbal_control.pitch_motor.current_set = gimbal_control.pitch_motor.current_set;
    } else if (gimbal_control.pitch_motor.last_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_control.pitch_motor.motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_control.pitch_motor.absolute_angle_set = gimbal_control.pitch_motor.absolute_angle;
    } else if (gimbal_control.pitch_motor.last_motor_mode != GIMBAL_MOTOR_GYRO_LIMIT && gimbal_control.pitch_motor.motor_mode == GIMBAL_MOTOR_GYRO_LIMIT) {
        gimbal_control.pitch_motor.absolute_angle_set = gimbal_control.pitch_motor.absolute_angle;
    } else if (gimbal_control.pitch_motor.last_motor_mode != GIMBAL_MOTOR_ENCONDE_LIMIT && gimbal_control.pitch_motor.motor_mode == GIMBAL_MOTOR_ENCONDE_LIMIT) {
        gimbal_control.pitch_motor.relative_angle_set = gimbal_control.pitch_motor.relative_angle;
    }
    gimbal_control.pitch_motor.last_motor_mode = gimbal_control.pitch_motor.motor_mode;
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 *
 * @retval         none
 */
static void gimbal_set_control(void)
{
    fp32 add_yaw_angle   = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);

    if (gimbal_motor_set_control_func[gimbal_control.pitch_motor.motor_mode] != NULL)
        gimbal_motor_set_control_func[gimbal_control.pitch_motor.motor_mode](&gimbal_control.pitch_motor, add_pitch_angle);

    if (gimbal_motor_set_control_func[gimbal_control.yaw_motor.motor_mode] != NULL)
        gimbal_motor_set_control_func[gimbal_control.yaw_motor.motor_mode](&gimbal_control.yaw_motor, add_yaw_angle);
}

static void gimbal_motor_raw_set_control(gimbal_motor_t *gimbal_motor, fp32 raw)
{
    if (gimbal_motor == NULL) {
        return;
    }

    gimbal_motor->raw_cmd_current = raw;
}

static void gimbal_motor_gyro_set_control(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL) {
        return;
    }

    gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}

static void gimbal_motor_gyro_limit_set_control(gimbal_motor_t *gimbal_motor, fp32 add)
{
    fp32 bias_angle;

    if (gimbal_motor == NULL) {
        return;
    }
    // now angle error
    // 当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    // relative angle + angle error + add_angle > max_relative angle
    // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle) {
        // calculate max add_angle
        // 计算出一个最大的添加角度，
        add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    } else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle) {

        add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }

    gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}

static void gimbal_motor_encoder_limit_set_control(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL) {
        return;
    }

    gimbal_motor->relative_angle_set += add;

    // 是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle) {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    } else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle) {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 *
 * @retval         none
 */
static void gimbal_control_loop(void)
{
    if (gimbal_motor_control_func[gimbal_control.yaw_motor.motor_mode] != NULL)
        gimbal_motor_control_func[gimbal_control.yaw_motor.motor_mode](&gimbal_control.yaw_motor);

    if (gimbal_motor_control_func[gimbal_control.pitch_motor.motor_mode] != NULL)
        gimbal_motor_control_func[gimbal_control.pitch_motor.motor_mode](&gimbal_control.pitch_motor);

    //* 根据电机安装方式设置控制值是否反向
    gimbal_control.pitch_motor.current_set *= PITCH_MOTOR_DIR;
    gimbal_control.yaw_motor.current_set *= YAW_MOTOR_DIR;
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL) {
        return;
    }

#if defined GIMBAL_DEBUG_OPEN && defined GIMBAL_DEBUG_INPUT_CODE
    gimbal_motor->raw_cmd_current = gimbal_debug_input();
#endif

    gimbal_motor->current_set = (int16_t)gimbal_motor->raw_cmd_current;
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL) {
        return;
    }

#if defined GIMBAL_DEBUG_ABSOLUTE_ANGEL && defined GIMBAL_DEBUG_INPUT_CODE
    gimbal_motor->absolute_angle_set = gimbal_debug_input();
#endif

    gimbal_motor->speed_set = PID_calc_specifyD(&gimbal_motor->absolute_angle_pid, rad_format(gimbal_motor->absolute_angle - gimbal_motor->absolute_angle_set), 0.0f, gimbal_motor->speed);

#if defined GIMBAL_DEBUG_SPEED && defined GIMBAL_DEBUG_INPUT_CODE
    gimbal_motor->speed_set = gimbal_debug_input();
#endif
    gimbal_motor->current_set = (int16_t)PID_calc(&gimbal_motor->speed_pid, gimbal_motor->speed, gimbal_motor->speed_set);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->speed_set   = PID_calc_specifyD(&gimbal_motor->relative_angle_pid, rad_format(gimbal_motor->relative_angle - gimbal_motor->relative_angle_set), 0.0f, gimbal_motor->speed);
    gimbal_motor->current_set = (int16_t)PID_calc(&gimbal_motor->speed_pid, gimbal_motor->speed, gimbal_motor->speed_set);
}

static fp32 gimbal_motor_yaw_speed_compensate(pid_typedef *pid)
{
    fp32 fric_compensation = 0;
    fp32 feedforward       = 0;

    // if (ABS(pid->set) > YAW_FRIC_COMPENSATION_DEADBAND) {
    //     fric_compensation = pid->set * YAW_FRIC_COMPENSATION_GAIN;
    //     ABS_LIMIT(fric_compensation, YAW_FRIC_COMPENSATION_MAX);
    // }

    // feedforward = pid->set / YAW_FEEDFORWARD_GAIN;

    return fric_compensation + feedforward;
}

#ifdef GIMBAL_DEBUG_INPUT_CODE
/**
 * @brief 作为云台调试控制值控制输入
 *
 * @return fp32
 */
static fp32 gimbal_debug_input(void)
{
    fp32 input = 0;

    //* 示例 1 产生恒定值输入 BEGIN
    // input = 15.0f;
    //* 示例 1 产生恒定值输入 END

    //* 示例 2 产生方波输入 BEGIN
    // const uint32_t period = 5000;                                   // 控制周期 | 单位 ms
    // uint32_t t            = (uint32_t)xTaskGetTickCount() % period; // 时间 | 单位 ms | 范围 0~period | tick 约 50 天才会溢出一次，不用担心

    // if (t < (period / 2)) {
    //     input = 0.0f;
    // } else {
    //     input = 10.0f;
    // }
    //* 示例 2 产生方波输入 END

    //* 示例 3 产生频率改变的正弦波输入 BEGIN
    uint32_t t           = (uint32_t)xTaskGetTickCount();
    const fp32 f[]       = {0.5f, 0.75f, 1.0f, 1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.25f, 2.5f, 2.75f, 3.0f, 3.25f, 3.5f, 4.0f, 4.5f, 5.0f, 5.5f, 6.0f, 6.5f, 7.0f, 7.5f, 8.0f, 8.5f, 9.0f, 9.5f, 10.0f, 10.5f, 11.0f, 11.5f, 12.0f, 12.5f, 13.0f, 13.5f, 14.0f, 14.5f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f, 22.0f, 24.0f, 26.0f, 28.0f, 30.0f, 32.0f, 34.0f, 36.0f, 38.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f, 110.0f, 120.0f, 140.0f, 160.0f, 180.0f, 200.0f, 225.0f, 250.0f};
    const uint16_t f_len = sizeof(f) / sizeof(fp32);
    static fp32 f_format = 1.0f;

    static uint16_t i        = 0;
    static uint16_t i_format = 0;

    static uint32_t t_start  = 0;
    static uint32_t t_need   = 2000.0f;
    static uint32_t t_passed = 0;

    if (t < 28000) {
        const uint32_t period = 4000;                                   // 控制周期 | 单位 ms
        uint32_t t1           = (uint32_t)xTaskGetTickCount() % period; // 时间 | 单位 ms | 范围 0~period | tick 约 50 天才会溢出一次，不用担心

        if (t1 < (period / 2)) {
            input = 0.2f;
        } else {
            input = -0.4f;
        }
    } else if (t < 31000) {
        input = -0.1f;
    } else {
        if (t_start == 0) t_start = t;

        t_passed = t - t_start;

        if (t_passed >= t_need) {
            i++;
            if (i >= f_len * 2) i = 0;
            i_format = i >= f_len ? (2 * f_len - i - 1) : i;

            t_start  = (uint32_t)xTaskGetTickCount();
            t_passed = 0;
            t_need   = 1000.0f / f[i_format];
            f_format = 1000.0f / t_need;
            if (i_format < 10)
                t_need *= 3;
            else if (i_format < 20)
                t_need *= 5;
            else if (i_format < 40)
                t_need *= 10;
            else
                t_need *= 20;
        }

        input = -0.1f + 0.3f * (arm_sin_f32(rad_format(2 * PI * f_format * t_passed / 1000.0f)));
    }
    //* 示例 3 产生频率改变的正弦波输入 END

    //* 示例 4 产生阶梯输入 BEGIN
    // static uint32_t t;
    // t = xTaskGetTickCount();

    // input = (t / 2000u) * 1000u;

    // if (input > 30000u) input = 60000u - input;
    //* 示例 4 产生阶梯输入 END

    return input;
}
#endif

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.pitch_motor;
}
