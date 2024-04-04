#include "cmsis_os.h"
#include "bsp_usart.h"
#include "auto_aim_task.h"
#include "usart.h"
#include "INS_task.h"
#include "string.h"
#include "math.h"
#include "UI.h"
#include "referee.h"


SentPacketTpye SentPacket;
RecevPackeType auto_aim_Packet;
AutoAimType AutoAimData;
const fp32 *INS_angle_point;

void auto_aim_init();
void updata_imu_angle();
void Read_robot_color();
float math_pi = 3.1415926;


void auto_aim_task(void const * argument)
{
    Read_robot_color();
    auto_aim_init();
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));

    while(1)
    {
        Read_robot_color();
        updata_imu_angle();
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&SentPacket, sizeof(SentPacket));

        osDelay(20); //刷新率=50Hz
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&auto_aim_Packet, sizeof(auto_aim_Packet));
        //超时处理，一定时间没触发接收中断判定NUC离线
        AutoAimData.timeout_count++;
        if(AutoAimData.timeout_count >= AUTOAIM_TIMEOUT_COUNT){
            AutoAimData.auto_aim_status = AUTOAIM_OFFLINE;
            AutoAimData.timeout_count = AUTOAIM_TIMEOUT_COUNT + 100; //防止计数器溢出
        }
    }
}

// 从裁判系统读取己方机器人颜色
void Read_robot_color()
{
    AutoAimData.self_robot_id = get_robot_id();
    if(AutoAimData.self_robot_id == UI_Data_RobotID_BHero){
        SentPacket.detect_color = 0; //red
    }else if(AutoAimData.self_robot_id == UI_Data_RobotID_RHero){
        SentPacket.detect_color = 1; //blue
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        //更新自瞄状态机&传递参数
        if((auto_aim_Packet.header == 0xA5 && !isnan(auto_aim_Packet.yaw) && !isnan(auto_aim_Packet.pitch)) &&
                (auto_aim_Packet.yaw <= M_PI &&  auto_aim_Packet.yaw >= -M_PI) && 
                (auto_aim_Packet.pitch <= M_PI &&  auto_aim_Packet.pitch >= -M_PI)){
            AutoAimData.auto_aim_status = AUTOAIM_LOCKED;
            AutoAimData.yaw = auto_aim_Packet.yaw;
            AutoAimData.pitch = auto_aim_Packet.pitch;
        }else{
            AutoAimData.auto_aim_status = AUTOAIM_LOST;
        }

        //清空超时计数
        AutoAimData.timeout_count = 0;
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
    SentPacket.detect_color = 0;
    SentPacket.reserved = 0;
    SentPacket.reset_tracker =0;
    /*INS指针初始化*/
    INS_angle_point = get_INS_angle_point();
    /*接收结构体初始化*/
    memset(&AutoAimData, 0, sizeof(AutoAimData));
    AutoAimData.auto_aim_status = AUTOAIM_LOST;
    AutoAimData.timeout_count = AUTOAIM_TIMEOUT_COUNT;
}

void updata_imu_angle()
{
    SentPacket.yaw = *(INS_angle_point + INS_ANGLE_YAW_ADDRESS_OFFSET);
    SentPacket.pitch = *(INS_angle_point + INS_ANGLE_PITCH_ADDRESS_OFFSET);
    SentPacket.roll = *(INS_angle_point + INS_ANGLE_ROLL_ADDRESS_OFFSET);
}
