//#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdbool.h"

typedef struct
{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}__attribute__((packed)) SentPacketTpye;

extern void auto_aim_task(void const * argument);