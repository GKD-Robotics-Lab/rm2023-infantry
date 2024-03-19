#include "superC_can_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include "cmsis_os.h"
#include "bsp_usart.h"

uint8_t superC_power_remaining;
uint8_t superC_bat_remaining;

void superC_can_task(void const * argument)
{
    uint16_t chassis_power_limit;

    for (;;)
    {
        get_chassis_power_limit(&chassis_power_limit);
        CAN_cmd_superC(chassis_power_limit);

        osDelay(100);
    }
}





