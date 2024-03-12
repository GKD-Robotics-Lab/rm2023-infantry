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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"
#include "superC_can_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// motor data read
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd      = (ptr)->ecd;                             \
        (ptr)->ecd           = (uint16_t)((data)[0] << 8 | (data)[1]); \
        (ptr)->speed_rpm     = (uint16_t)((data)[2] << 8 | (data)[3]); \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate     = (data)[6];                              \
    }

// 电机报文数据，数组元素对应的电机见枚举 motor_id_e
static motor_measure_t motor_chassis[MOTOR_NUM];

// 电机发送数据的 buffer
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t shoot_can_send_data[8];
static CAN_TxHeaderTypeDef superC_tx_message;
static uint8_t superC_can_send_data[2];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId) {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID: {
            static uint8_t i = 0;
            // get motor id
            i = rx_header.StdId - (CAN_CHASSIS_ALL_ID + 1);
            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }

        case CAN_SUPERC_RX_ID: {
            superC_power_remaining = rx_data[0];
            superC_bat_remaining = rx_data[1];
        }

        default:
            break;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    switch (rx_header.StdId) {
        case CAN_FRIC_M1_ID:
            detect_hook(SHOOT_FRIC1_TOE);
            get_motor_measure(&motor_chassis[MOTOR_SHOOT_FRIC1_ID], rx_data);
            break;
        case CAN_FRIC_M2_ID:
            detect_hook(SHOOT_FRIC2_TOE);
            get_motor_measure(&motor_chassis[MOTOR_SHOOT_FRIC2_ID], rx_data);
            break;
        case CAN_TRIGGER_ID:
            detect_hook(SHOOT_TRIGGER_TOE);
            get_motor_measure(&motor_chassis[MOTOR_SHOOT_TRIGGER_ID], rx_data);
            break;
        case CAN_YAW_MOTOR_ID:
            detect_hook(YAW_GIMBAL_MOTOR_TOE);
            get_motor_measure(&motor_chassis[MOTOR_GIMBAL_YAW_ID], rx_data);
            break;
        case CAN_PIT_MOTOR_ID:
            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            get_motor_measure(&motor_chassis[MOTOR_GIMBAL_PITCH_ID], rx_data);
            break;
        default:
            break;
    }
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE   = CAN_ID_STD;
    gimbal_tx_message.RTR   = CAN_RTR_DATA;
    gimbal_tx_message.DLC   = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (rev1 >> 8);
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8);
    gimbal_can_send_data[7] = rev2;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t fric1, int16_t fric2, int16_t trigger, int16_t rev)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE   = CAN_ID_STD;
    shoot_tx_message.RTR   = CAN_RTR_DATA;
    shoot_tx_message.DLC   = 0x08;
    shoot_can_send_data[0] = (fric1 >> 8);
    shoot_can_send_data[1] = fric1;
    shoot_can_send_data[2] = (fric2 >> 8);
    shoot_can_send_data[3] = fric2;
    shoot_can_send_data[4] = (trigger >> 8);
    shoot_can_send_data[5] = trigger;
    shoot_can_send_data[6] = (rev >> 8);
    shoot_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE   = CAN_ID_STD;
    chassis_tx_message.RTR   = CAN_RTR_DATA;
    chassis_tx_message.DLC   = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE   = CAN_ID_STD;
    chassis_tx_message.RTR   = CAN_RTR_DATA;
    chassis_tx_message.DLC   = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_superC(uint16_t power_limit)
{
    uint32_t send_mail_box;
    superC_tx_message.StdId = 0xFE;
    superC_tx_message.IDE   = CAN_ID_STD;
    superC_tx_message.RTR   = CAN_RTR_DATA;
    superC_tx_message.DLC   = 0x02;
    superC_can_send_data[0] = power_limit >> 8;
    superC_can_send_data[1] = power_limit;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &superC_tx_message, superC_can_send_data, &send_mail_box);
}
/**
 * @brief          返回指定的电机报文数据指针
 * @param[in]      id
 * @retval         电机数据指针
 */
const motor_measure_t *get_motor_measure_point(motor_id_e id)
{
    return &motor_chassis[id];
}
