/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN  hcan2
#define SHOOT_CAN   hcan2

/*
  > 当前 CAN 总线分配：
  >     CAN1：-> FIFO0
  >         底盘电机
  >             3508 0x201~0x204
  >     CAN2：-> FIFO1
  >         射击控制电机
  >             摩擦轮 3508 0x201, 0x202
  >             拨弹轮 2006 0x203
  >         云台电机
  >             YAW   6020 0x205
  >             PITCH 6020 0x206
  */

/* CAN send and receive ID */
typedef enum {
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID     = 0x201,
    CAN_3508_M2_ID     = 0x202,
    CAN_3508_M3_ID     = 0x203,
    CAN_3508_M4_ID     = 0x204,

    // CAN2
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID  = 0x205,
    CAN_PIT_MOTOR_ID  = 0x206,

    CAN_SHOOT_ALL_ID = 0x200,
    CAN_FRIC_M1_ID   = 0x201,
    CAN_FRIC_M2_ID   = 0x202,
    CAN_TRIGGER_ID   = 0x203,

    CAN_SUPERC_RX_ID = 0x0FE,
    CAN_SUPERC_TX_ID = 0x0FF,

} can_msg_id_e;

typedef enum {
    MOTOR_CHASSIS_1_ID = 0,
    MOTOR_CHASSIS_2_ID, // 1
    MOTOR_CHASSIS_3_ID, // 2
    MOTOR_CHASSIS_4_ID, // 3

    MOTOR_GIMBAL_YAW_ID,   // 4
    MOTOR_GIMBAL_PITCH_ID, // 5

    MOTOR_SHOOT_FRIC1_ID,   // 6
    MOTOR_SHOOT_FRIC2_ID,   // 7
    MOTOR_SHOOT_TRIGGER_ID, // 8

    MOTOR_NUM, // 电机总个数，也是电机报文数组的长度
} motor_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);
extern void CAN_cmd_shoot(int16_t trigger, int16_t fric1, int16_t fric2, int16_t rev);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_superC(uint16_t power_limit);
extern const motor_measure_t *get_motor_measure_point(motor_id_e i);

#endif
