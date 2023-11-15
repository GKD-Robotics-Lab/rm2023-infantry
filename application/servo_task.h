#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

//盖板控制通道（遥控器左上滚轮）
#define COVER_PLATE_CHANNEL 4

extern void servo_task(void const * argument);

#endif
