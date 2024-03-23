#include "cmsis_os.h"
#include "bsp_usart.h"
#include "auto_aim_task.h"
#include "usart.h"
#include "INS_task.h"

SentPacketTpye SentPacket;
const fp32 *INS_angle;
const fp32 *INS_gyro;

void auto_aim_task(void const * argument)
{
    SentPacket.header = 0x5A;
    SentPacket.checksum = 0;
    SentPacket.aim_x = 0.0;
    SentPacket.aim_y = 0.0;
    SentPacket.aim_z = 0.0;
    SentPacket.detect_color = 0;
    SentPacket.reserved = 0;
    SentPacket.reset_tracker =0;

    INS_angle = get_INS_angle_point();
    INS_gyro  = get_gyro_data_point();


    while(1)
    {
        SentPacket.yaw = INS_angle + ;
        SentPacket.roll = 2.0;
        SentPacket.pitch = 3.0;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&SentPacket, sizeof(SentPacket));

        osDelay(2); //刷新率=500Hz
    }
}