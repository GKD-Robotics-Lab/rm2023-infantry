/**
 * @file shoot_task.c
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "shoot_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

// 为精简代码将遥控器相关按键使用宏替代
#define RC_shoot_switch (shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) // 遥控器射击控制拨杆
#define RC_mouse_l      (shoot_control.shoot_rc->mouse.press_l)               // 鼠标左键
#define RC_mouse_r      (shoot_control.shoot_rc->mouse.press_r)               // 鼠标右键

static void shoot_init(void);
static void shoot_set_mode(void);
static void shoot_feedback_update(void);
static void shoot_switch_mode(void);
static void shoot_control_loop(void);

static shoot_control_t shoot_control; // 射击控制结构体

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t shoot_high_water;
#endif

void shoot_task(void const *pvParameters)
{
    //* wait a time
    // 空闲一段时间
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    //* shoot init
    // 射击初始化
    shoot_init();

    //* detect
    // make sure all shoot motor is online,
    // 判断射击控制电机是否都在线
    while (toe_is_error(SHOOT_TRIGGER_TOE) || toe_is_error(SHOOT_FRIC1_TOE) || toe_is_error(SHOOT_FRIC2_TOE) || toe_is_error(DBUS_TOE)) {
        vTaskDelay(SHOOT_CONTROL_TIME);
    }

    while (1) {
        //* set shoot control mode
        // 根据遥控器和键盘设置射击控制模式
        shoot_set_mode();
        //* shoot data update
        // 射击数据更新
        shoot_feedback_update();
        //* switch shoot control mode
        // 根据反馈切换射击控制模式
        shoot_switch_mode();
        //* shoot control value calculate
        // 射击控制输出值计算
        shoot_control_loop();

        //* detect
        // make sure  one motor is online at least, so that the control CAN message can be received
        // 确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(SHOOT_TRIGGER_TOE) && toe_is_error(SHOOT_FRIC1_TOE) && toe_is_error(SHOOT_FRIC2_TOE))) {
            // when remote control is offline, shoot motor should receive zero current.
            // 当遥控器掉线的时候，发送给射击电机零电流.
            if (toe_is_error(DBUS_TOE)) {
                CAN_cmd_shoot(0, 0, 0, 0);
            } else {
                CAN_cmd_shoot(shoot_control.fric1.current_set, shoot_control.fric2.current_set, shoot_control.trigger.current_set, 0);
            }
        }

        // os delay
        // 系统延时
        vTaskDelay(SHOOT_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief 射击初始化
 *
 */
static void shoot_init(void)
{
    static const fp32 fric_speed_pid[3]    = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    static const fp32 trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};

    //* 射击模式初始化
    shoot_control.shoot_mode = SHOOT_DISABLE;

    //* 获取遥控器及电机报文指针
    shoot_control.shoot_rc              = get_remote_control_point();
    shoot_control.fric1.motor_measure   = get_motor_measure_point(MOTOR_SHOOT_FRIC1_ID);
    shoot_control.fric2.motor_measure   = get_motor_measure_point(MOTOR_SHOOT_FRIC2_ID);
    shoot_control.trigger.motor_measure = get_motor_measure_point(MOTOR_SHOOT_TRIGGER_ID);

    //* 初始化 PID 和斜坡函数
    PID_init(&shoot_control.fric1.motor_pid, PID_DELTA, fric_speed_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_DEAD_BAND);
    PID_init(&shoot_control.fric2.motor_pid, PID_DELTA, fric_speed_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_DEAD_BAND);
    PID_init(&shoot_control.trigger.speed_pid, PID_DELTA, trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_DEAD_BAND);
    ramp_init(&shoot_control.fric_ramp, SHOOT_CONTROL_TIME * 0.001, FRIC_SPEED_FULL, FRIC_SPEED_STOP);

    //* 射击控制其他参数初始化
    shoot_control.fric1.pspeed_set  = &shoot_control.fric_ramp.out;
    shoot_control.fric2.pspeed_set  = &shoot_control.fric_ramp.out;
    shoot_control.trigger.ecd_count = 0;

    shoot_control.trigger.block_time   = 0;
    shoot_control.trigger.reverse_time = 0;

    //* 更新一次反馈数据
    shoot_feedback_update();
}

/**
 * @brief 射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 *
 */
static void shoot_set_mode(void)
{
    static int8_t last_sw        = RC_SW_UP; // 记录上次拨杆位置
    static uint8_t last_ml       = 0;        // 记录上次鼠标左键
    static uint8_t last_mr       = 0;        // 记录上次鼠标右键
    static uint32_t sw_down_time = 0;
    static uint32_t press_l_time = 0;

    //! 摩擦轮开关控制
    //* 上拨判断，一次开启，再次关闭
    if ((switch_is_up(RC_shoot_switch) && !switch_is_up(last_sw) && (shoot_control.shoot_mode == SHOOT_STOP || shoot_control.shoot_mode == SHOOT_DISABLE))) {
        // 重设 last_angle
        shoot_control.trigger.last_angle = shoot_control.trigger.angle;
        shoot_control.shoot_mode         = SHOOT_START;
        // 设置斜坡函数为加速
        shoot_control.fric_ramp.input = FRIC_RAMP_ADD;
    } else if ((switch_is_up(RC_shoot_switch) && !switch_is_up(last_sw) && shoot_control.shoot_mode != SHOOT_STOP && shoot_control.shoot_mode != SHOOT_DISABLE)) {
        shoot_control.shoot_mode = SHOOT_STOP;
        // 设置斜坡函数为减速
        shoot_control.fric_ramp.input = FRIC_RAMP_SUB;
    }

    //* 如果云台状态是 无力状态，就关闭射击
    if(gimbal_cmd_to_shoot_stop()) {
        shoot_control.shoot_mode = SHOOT_STOP;
        shoot_control.fric_ramp.input = FRIC_RAMP_SUB;
    }

    //* 处于中档，可以使用键盘开启和关闭摩擦轮
    if (switch_is_mid(RC_shoot_switch) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP) {
        shoot_control.shoot_mode = SHOOT_START;
    } else if (switch_is_mid(RC_shoot_switch) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP) {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //! 射击控制
    //* 下拨一次或者鼠标按下一次，进入射击状态
    if (shoot_control.shoot_mode == SHOOT_READY) {
        if ((switch_is_down(RC_shoot_switch) && !switch_is_down(last_sw)) || (RC_mouse_l && last_ml == 0) || (RC_mouse_r && last_mr == 0)) {
            shoot_control.shoot_mode = SHOOT_FIRE;
        }
    }

    //* 鼠标左键长按进入射击连发状态，松开后退出连发
    if (shoot_control.shoot_mode == SHOOT_READY || shoot_control.shoot_mode == SHOOT_FIRE || shoot_control.shoot_mode == SHOOT_CONTINUE_FIRE || shoot_control.shoot_mode == SHOOT_DONE) {
        if ((press_l_time == PRESS_LONG_TIME) || (sw_down_time == RC_SW_LONG_TIME)) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_FIRE;
        } else if (shoot_control.shoot_mode == SHOOT_CONTINUE_FIRE) {
            shoot_control.shoot_mode = SHOOT_DONE;
        }
    }

    //* 鼠标右键按下加速摩擦轮
    // TODO 是不是应该加速拨弹轮
    if (RC_mouse_r)
        shoot_control.fric_ramp.max_value = FRIC_SPEED_TURBO;
    else
        shoot_control.fric_ramp.max_value = FRIC_SPEED_FULL;

    //! 考虑枪口热量限制及云台未启动
    //* 枪口热量控制
    // TODO 右键加速射击时不进行热量限制？
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit)) {
        if (shoot_control.shoot_mode == SHOOT_FIRE || shoot_control.shoot_mode == SHOOT_CONTINUE_FIRE) {
            shoot_control.shoot_mode = SHOOT_READY;
        }
    }

    //! 控制计时
    //* 左键长按计时
    if (RC_mouse_l) {
        if (press_l_time < PRESS_LONG_TIME) press_l_time++;
    } else {
        press_l_time = 0;
    }

    //* 射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(RC_shoot_switch)) {
        if (sw_down_time < RC_SW_LONG_TIME) {
            sw_down_time++;
        }
    } else {
        sw_down_time = 0;
    }

    //* 遥控器上次状态记录
    last_sw = RC_shoot_switch;
    last_ml = RC_mouse_l;
    last_mr = RC_mouse_r;
}

/**
 * @brief 射击数据更新
 *
 */
static void shoot_feedback_update(void)
{
    // 拨弹轮电机速度滤波一下 // 二阶低通滤波
    // TODO 看一下这个滤波
    static fp32 trigger_speed[3]                  = {0.0f};
    static const fp32 trigger_speed_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //* 拨弹轮速度获取
    trigger_speed[2]            = trigger_speed[1];
    trigger_speed[1]            = trigger_speed[0];
    trigger_speed[0]            = trigger_speed[1] * trigger_speed_fliter_num[0] + trigger_speed[2] * trigger_speed_fliter_num[1] + rpm_to_radps(shoot_control.trigger.motor_measure->speed_rpm) * trigger_speed_fliter_num[2] / TRIGGER_MOTOR_REDUCTION_RATIO_3508;
    shoot_control.trigger.speed = trigger_speed[0];

    //* 拨弹轮角度获取
    // 电机圈数重置，因为输出轴旋转一圈，电机轴旋转 36 圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.trigger.motor_measure->ecd - shoot_control.trigger.motor_measure->last_ecd > HALF_ECD_RANGE) {
        shoot_control.trigger.ecd_count--;
    } else if (shoot_control.trigger.motor_measure->ecd - shoot_control.trigger.motor_measure->last_ecd < -HALF_ECD_RANGE) {
        shoot_control.trigger.ecd_count++;
    }

    if (shoot_control.trigger.ecd_count == FULL_ECD_COUNT_3508) {
        shoot_control.trigger.ecd_count = -(FULL_ECD_COUNT_3508 - 1);
    } else if (shoot_control.trigger.ecd_count == -FULL_ECD_COUNT_3508) {
        shoot_control.trigger.ecd_count = FULL_ECD_COUNT_3508 - 1;
    }

    // 计算输出轴角度
    shoot_control.trigger.angle = (shoot_control.trigger.ecd_count * ECD_RANGE + shoot_control.trigger.motor_measure->ecd) * TRIGGER_MOTOR_ECD_TO_ANGLE;

    //* 摩擦轮速度获取
    shoot_control.fric1.speed = rpm_to_radps(shoot_control.fric1.motor_measure->speed_rpm);
    shoot_control.fric2.speed = rpm_to_radps(shoot_control.fric2.motor_measure->speed_rpm);
}

/**
 * @brief 根据反馈切换射击控制模式
 *
 */
static void shoot_switch_mode(void)
{
    static uint32_t done_time = 0;
    switch (shoot_control.shoot_mode) {
        case SHOOT_START:
            if (shoot_control.fric_ramp.out >= shoot_control.fric_ramp.max_value)
                shoot_control.shoot_mode = SHOOT_READY;
            break;
        case SHOOT_FIRE:
            if (rad_format(shoot_control.trigger.angle - shoot_control.trigger.last_angle) > 2 * PI / PIT_NUM_HERO)
                shoot_control.shoot_mode = SHOOT_DONE;
            break;
        case SHOOT_DONE:
            done_time++;
            if (done_time > SHOOT_DONE_KEY_OFF_TIME) {
                done_time                = 0;
                shoot_control.shoot_mode = SHOOT_READY;
                // 退出 DONE 时记录角度
                shoot_control.trigger.last_angle = shoot_control.trigger.angle;
            }
            break;
        case SHOOT_STOP:
            if (shoot_control.fric_ramp.out <= shoot_control.fric_ramp.min_value)
                shoot_control.shoot_mode = SHOOT_DISABLE;
            break;
        default:
            break;
    }
}

/**
 * @brief 根据不同的射击控制模式计算电机控制值
 *
 */
static void shoot_control_loop(void)
{
    //* 摩擦轮速度控制
    if (shoot_control.shoot_mode == SHOOT_DISABLE) {
        // 相关电机均无力
        shoot_control.fric1.current_set = 0.0f;
        shoot_control.fric2.current_set = 0.0f;
    } else {
        // 使用斜坡函数的计算输出值作为速度
        ramp_calc(&shoot_control.fric_ramp, shoot_control.fric_ramp.input);
#if FRIC_DIRECTION
        shoot_control.fric1.current_set = PID_calc(&shoot_control.fric1.motor_pid, shoot_control.fric1.speed, *shoot_control.fric1.pspeed_set);
        shoot_control.fric2.current_set = PID_calc(&shoot_control.fric2.motor_pid, shoot_control.fric2.speed, -*shoot_control.fric2.pspeed_set);
#else
        shoot_control.fric1.current_set = PID_calc(&shoot_control.fric1.motor_pid, shoot_control.fric1.speed, -*shoot_control.fric1.pspeed_set);
        shoot_control.fric2.current_set = PID_calc(&shoot_control.fric2.motor_pid, shoot_control.fric2.speed, *shoot_control.fric2.pspeed_set);
#endif
    }

    //* 拨弹轮速度控制
    switch (shoot_control.shoot_mode) {
        case SHOOT_DISABLE:
            // 相关电机均无力
            shoot_control.trigger.current_set = 0.0f;
            break;
        case SHOOT_START:
        case SHOOT_READY:
        case SHOOT_DONE:
        case SHOOT_STOP:
            // 拨弹轮不动
            shoot_control.trigger.speed_set   = 0.0f;
            shoot_control.trigger.current_set = PID_calc(&shoot_control.trigger.speed_pid, shoot_control.trigger.speed, shoot_control.trigger.speed_set);
            break;
        case SHOOT_FIRE:
        case SHOOT_CONTINUE_FIRE:
            // 拨弹轮转动，并进行堵转检测
            if (shoot_control.trigger.block_time < TRIGGER_BLOCK_TIME) {
                shoot_control.trigger.speed_set = shoot_control.shoot_mode == SHOOT_FIRE ? TRIGGER_SPEED : TRIGGER_CONTINUE_FIRE_SPEED;
            } else {
                shoot_control.trigger.speed_set = TRIGGER_REVERSE_SPEED;
            }

            if (ABS(shoot_control.trigger.speed) < TRIGGER_BLOCK_SPEED && shoot_control.trigger.block_time < TRIGGER_BLOCK_TIME) {
                shoot_control.trigger.block_time++;
                shoot_control.trigger.reverse_time = 0;
            } else if (shoot_control.trigger.block_time >= TRIGGER_BLOCK_TIME && shoot_control.trigger.reverse_time < TRIGGER_REVERSE_TIME) {
                shoot_control.trigger.reverse_time++;
            } else {
                shoot_control.trigger.block_time = 0;
            }

            shoot_control.trigger.current_set = PID_calc(&shoot_control.trigger.speed_pid, shoot_control.trigger.speed, shoot_control.trigger.speed_set);
            break;
    }
}
