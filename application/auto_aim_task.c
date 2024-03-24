#include "cmsis_os.h"
#include "bsp_usart.h"
#include "auto_aim_task.h"
#include "usart.h"
#include "INS_task.h"
#include "string.h"

SentPacketTpye SentPacket;
RecevPackeType auto_aim_Packet;
AutoAimType AutoAimData;
const fp32 *INS_angle_point;

void auto_aim_init();
void updata_imu_angle();


void auto_aim_task(void const * argument)
{
    auto_aim_init();

    while(1)
    {
        updata_imu_angle();
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&SentPacket, sizeof(SentPacket));

        HAL_UART_Receive_DMA(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));
        //更新自瞄状态机&传递参数
        if(auto_aim_Packet.header == 0xA5){
            AutoAimData.auto_aim_status = AUTOAIM_LOCKED;
            AutoAimData.yaw = auto_aim_Packet.yaw;
            AutoAimData.pitch = auto_aim_Packet.pitch;
        }else{
            AutoAimData.auto_aim_status = AUTOAIM_LOST;
        }
        memset(&AutoAimData, 0, sizeof(AutoAimData));   //清空结构体

        //usart6_printf("TEST\n");

        osDelay(2); //刷新率=500Hz
    }
}

void auto_aim_init()
{
    /*SendPacket初始化*/
    SentPacket.header = 0x5A;
    SentPacket.checksum = 0;
    SentPacket.aim_x = 0.0;
    SentPacket.aim_y = 0.0;
    SentPacket.aim_z = 0.0;
    SentPacket.detect_color = 1;
    SentPacket.reserved = 0;
    SentPacket.reset_tracker =0;
    /*INS指针初始化*/
    INS_angle_point = get_INS_angle_point();
    /*接收结构体初始化*/
    memset(&AutoAimData, 0, sizeof(AutoAimData));
    AutoAimData.auto_aim_status = AUTOAIM_LOST;
}

void updata_imu_angle()
{
    SentPacket.yaw = *(INS_angle_point + INS_ANGLE_YAW_ADDRESS_OFFSET);
    SentPacket.pitch = *(INS_angle_point + INS_ANGLE_PITCH_ADDRESS_OFFSET);
    SentPacket.roll = *(INS_angle_point + INS_ANGLE_ROLL_ADDRESS_OFFSET);
}