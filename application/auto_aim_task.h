//#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdbool.h"

#define AUTOAIM_LOST 0
#define AUTOAIM_LOCKED 1

#define AUTOAIM_TIMEOUT 4     
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

typedef struct
{
  uint8_t header; //0xA5
  float yaw;
  float pitch;
}__attribute__((packed)) RecevPackeType;

typedef struct
{
  int timeout_count;    //超时计数
  int auto_aim_status;  //自瞄状态（是否可用）
  float yaw;
  float pitch;
}AutoAimType;

extern void auto_aim_task(void const * argument);
extern AutoAimType AutoAimData;