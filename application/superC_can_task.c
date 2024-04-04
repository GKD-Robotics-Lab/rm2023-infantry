#include "superC_can_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "custom_ui_task.h"

uint8_t superC_power_remaining;
uint8_t superC_bat_remaining;

void superC_can_task(void const * argument)
{
    uint16_t chassis_power_limit = 55;

    for (;;)
    {
        get_chassis_power_limit(&chassis_power_limit);
        if (chassis_power_limit >= 40)
            CAN_cmd_superC(chassis_power_limit);
        else 
            CAN_cmd_superC(45);
            
        UI_Data.Super_cap_percent = superC_bat_remaining;

        osDelay(100);
    }
}





