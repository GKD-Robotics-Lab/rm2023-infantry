#include "cmsis_os.h"
#include "bsp_usart.h"
#include "auto_aim_task.h"
#include "usart.h"
#include "INS_task.h"
#include "string.h"
#include "math.h"


SentPacketTpye SentPacket;
RecevPackeType auto_aim_Packet;
AutoAimType AutoAimData;
const fp32 *INS_angle_point;

void auto_aim_init();
void updata_imu_angle();
float math_pi = 3.1415926;


void auto_aim_task(void const * argument)
{
    auto_aim_init();
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));

    while(1)
    {
        updata_imu_angle();
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&SentPacket, sizeof(SentPacket));

        //HAL_UART_Receive(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet), 0x02);
        //usart6_printf("head:%x, yaw:%f, pitch:%f\n", auto_aim_Packet.header, auto_aim_Packet.yaw, auto_aim_Packet.pitch);
        //usart6_printf("yaw:%f, pitch:%f, LOCK:%d\n", AutoAimData.yaw, AutoAimData.pitch, AutoAimData.auto_aim_status);
        //memset(&AutoAimData, 0, sizeof(AutoAimData));   //清空结构体
        osDelay(20); //刷新率=50Hz
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));
        AutoAimData.timeout_count ++;
        if(AutoAimData.timeout_count >= AUTOAIM_TIMEOUT) AutoAimData.auto_aim_status = AUTOAIM_LOST;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        //更新自瞄状态机&传递参数
        if((auto_aim_Packet.header == 0xA5 && !isnan(auto_aim_Packet.yaw) && !isnan(auto_aim_Packet.pitch)) &&
                (auto_aim_Packet.yaw <= math_pi &&  auto_aim_Packet.yaw >= -math_pi) &&
                (auto_aim_Packet.pitch <= math_pi &&  auto_aim_Packet.pitch >= -math_pi)){
            AutoAimData.auto_aim_status = AUTOAIM_LOCKED;
            AutoAimData.yaw = auto_aim_Packet.yaw;
            AutoAimData.pitch = auto_aim_Packet.pitch;
            AutoAimData.timeout_count = 0;      //超时清零
        }else{
            AutoAimData.auto_aim_status = AUTOAIM_LOST;
        }        //memset(&auto_aim_Packet, 0, sizeof(auto_aim_Packet));   //清空结构体
        //usart6_printf("yaw:%f, pitch:%f, LOCK:%d\n", AutoAimData.yaw, AutoAimData.pitch, AutoAimData.auto_aim_status);

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));
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
    AutoAimData.timeout_count = 0;
}

void updata_imu_angle()
{
    SentPacket.yaw = *(INS_angle_point + INS_ANGLE_YAW_ADDRESS_OFFSET);
    SentPacket.pitch = *(INS_angle_point + INS_ANGLE_PITCH_ADDRESS_OFFSET);
    SentPacket.roll = *(INS_angle_point + INS_ANGLE_ROLL_ADDRESS_OFFSET);
}