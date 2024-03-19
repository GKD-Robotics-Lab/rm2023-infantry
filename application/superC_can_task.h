#ifndef _SUPERC_CAN_TASK_H_
#define _SUPERC_CAN_TASK_H_

#include "struct_typedef.h"

extern uint8_t superC_power_remaining;
extern uint8_t superC_bat_remaining;


extern void superC_can_task(void const * argument);

#endif

