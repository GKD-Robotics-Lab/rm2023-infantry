#include "superC_usart_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include "cmsis_os.h"

void superC_usart_task(void const * argument)
{
    uint16_t chassis_power_limit;

    for (;;)
    {
        get_chassis_power_limit(&chassis_power_limit);
        CAN_cmd_superC(chassis_power_limit);

        osDelay(1000);
    }
}

