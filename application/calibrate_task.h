/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c/h
  * @brief      calibrate these device，include gimbal, gyro, accel, magnetometer,
  *             chassis. gimbal calibration is to calc the midpoint, max/min
  *             relative angle. gyro calibration is to calc the zero drift.
  *             accel and mag calibration have not been implemented yet, because
  *             accel is not necessary to calibrate, mag is not used. chassis
  *             calibration is to make motor 3508 enter quick reset ID mode.
  *             校准设备，包括云台,陀螺仪,加速度计,磁力计,底盘.云台校准是主要计算零点
  *             和最大最小相对角度.云台校准是主要计算零漂.加速度计和磁力计校准还没有实现
  *             因为加速度计还没有必要去校准,而磁力计还没有用.底盘校准是使M3508进入快速
  *             设置ID模式.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis clabration
  *
  @verbatim
  ==============================================================================
  * >           校准: 
  * >               方法一: 上电后检测到遥控器拨杆与摇杆均在下校准全部设备
  * >               方法二: 按下按键 1s 校准全部设备
  * >               方法三: 保持拨杆都在下，将摇杆内拨，听到提示音后即开始选择校准设备
  * >                       选择设备几就外拨几次，选择过程中与完成后均有提示音
  * >                       现有可选校准设备: 1->陀螺仪
  *
  * >           数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
  * >               例如head_cali有8个字节(不__packed的话应该是16字节)，但它需要12字节在flash，如果它从0x080A0000开始
  * >                   0x080A0000-0x080A0007: head_cali数据
  * >                   0x080A0008: 名字name[0]
  * >                   0x080A0009: 名字name[1]
  * >                   0x080A000A: 名字name[2]
  * >                   0x080A000B: 校准标志位 cali_flag -> 当校准标志位为0x55,意味着head_cali已经校准了
  *
  * >           添加新设备
  * >               1.添加设备名在 calibrate_task.h 的 cali_id_e, 像
  * >                   typedef enum
  * >                   {
  * >                       ...
  * >                       //add more...
  * >                       CALI_XXX,
  * >                       CALI_LIST_LENGHT,
  * >                   } cali_id_e;
  * >               2.添加数据结构在 calibrate_task.h, 必须 4 字节倍数，像
  * >                   typedef struct
  * >                   {
  * >                       uint16_t xxx;
  * >                       uint16_t yyy;
  * >                       fp32 zzz;
  * >                   } xxx_cali_t; // 长度: 8字节 8 bytes, 必须是 4, 8, 12, 16...
  * >               3.添加新校准设备所用的一系列变量
  * >                   添加新校准设备名称在 cali_name[CALI_LIST_LENGHT][3]
  * >                   申明校准数据变量 xxx_cali_t xxx_cail
  * >                   添加校准数据变量地址在 cali_data_address[CALI_LIST_LENGHT]
  * >                   在 cali_data_len_word[CALI_LIST_LENGHT] 添加数据长度 (word)
  * >                   在 cali_hook_fun[CALI_LIST_LENGHT]添加函数
  * >                   在 "FLASH_WRITE_BUF_LENGHT" 中添加校准数据长度 "sizeof(xxx_cali_t)"
  * >               4.实现新函数 bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd)
  * >                   这里 hook 函数的作用是 1.启动校准 2.将得到的校准值拿到设备那边用
  * >                   cmd 参数有两个：
  * >                       CALI_FUNC_CMD_INIT，已经校准过，直接使用校准值
  * >                       CALI_FUNC_CMD_ON，开始校准
  * >                   该函数在参数为 CALI_FUNC_CMD_INIT 的情况下返回 1 表示每次上电都需要校准，否则为不需要
  * >                   该函数在参数为 CALI_FUNC_CMD_ON 的情况下返回 1 表示校准完成，否则为未完成
  * >                   这些函数对应设置的校准值必须是在任务启动前就已经存在的静态变量
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"
#include "bsp_buzzer.h"

#define CALIBRATE_CONTROL_TIME      1 // osDelay time,  means 1ms.1ms 系统延时
#define CALIBRATE_CONTROL_INIT_TIME 57

//* calibrate
// cali_done
#define CALIED_FLAG 0x55 // means it has been calibrated
// cali func cmd
#define CALI_FUNC_CMD_ON   1 // need calibrate,设置校准
#define CALI_FUNC_CMD_INIT 0 // has been calibrated, set value to init.已经校准过，设置校准值
// cali param
#define CALI_EX_DATA_LEN_WORD 1 // 额外数据 name[3] 和 cali_done 的长度 (word)

//* buzzer
#define cali_all_start_buzzer()                buzzer_on(0, 30000)
#define cali_RC_choose_begin_start_buzzer()    buzzer_on(1, 25000)
#define cali_RC_choose_verified_start_buzzer() buzzer_on(2, 20000)
#define cali_RC_choose_OK_start_buzzer()       buzzer_on(0, 30000)
#define cali_imu_start_buzzer()                buzzer_on(95, 10000)
#define cali_buzzer_off()                      buzzer_off() // buzzer off，关闭蜂鸣器
// when imu is calibrating ,buzzer set frequency and strength. 当imu在校准,蜂鸣器的设置频率和强度
// #define imu_start_buzzer() buzzer_on(95, 10000)

// #define rc_cali_buzzer_start_on()  imu_start_buzzer()

//* gyro
#define GYRO_CALIBRATE_TIME 20000 // gyro calibrate time,陀螺仪校准时间

//* temperature
#define GYRO_CONST_MAX_TEMP 45.0f // max control temperature of gyro,最大陀螺仪控制温度

//* flash
#define FLASH_USER_ADDR                     ADDR_FLASH_SECTOR_9                                 // write flash page 9,保存的flash页地址
#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                 // flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) // flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))          // flash erase function,flash擦除函数

//* RC
// RC control
#define cali_disable_RC_control() RC_unable() // when imu is calibrating, disable the remote control.当imu在校准时候,失能遥控器
#define cali_enable_RC_control()  RC_restart(SBUS_RX_BUF_NUM)
// RC value limit
#define RC_CALI_VALUE_HOLE 600 // remote control threshold, the max value of remote control channel is 660.
// RC time set
#define RC_CMD_START_TIME  1500
#define RC_CMD_VERIFY_TIME 150
#define RC_CMD_STOP_TIME   1500

#define RC_FLAG_NO_CMD -1 // 遥控器没有接收到命令状态
#define RC_FLAG_BEGIN  0  // 进入遥控器选择状态
#define RC_FLAG_GYRO   1

//* key
#define KEY_LONG_TIME 1000 // 按键持续按下控制校准的时间
#define KEY_SET_CALI_DONT_CHECK 0 // 不检查是否已经有校准，适用于遥控器连接前等待
#define KEY_SET_CALI_CHECK 1 // 检查是否有在校准的

//* cali device structure
typedef struct
{
    uint8_t name[3];                         // device name
    uint8_t cali_done;                       // 0x55 means has been calibrated
    uint8_t data_len_word : 7;               // buf lenght
    uint8_t cali_cmd : 1;                    // 1 means to run cali hook function,
    uint32_t *data;                          // link to device calibration data
    bool_t (*cali_hook)(uint32_t *, bool_t); // cali function
} __packed cali_device_t;

//* cali device id
//// 其实这个 id 只是用来计算设备列表长度
typedef enum {
    CALI_TEMP = 0, // 设置陀螺仪的设定温度 temperature
    CALI_GYRO = 1, // 陀螺仪零漂校准
    CALI_ACC  = 2,
    CALI_MAG  = 3,
    //! STEP 1 添加设备名 BEGIN

    //! STEP 1 添加设备名 END
    CALI_LIST_LENGHT,
} cali_id_e;

//* cali data structure
// temperature
typedef struct
{
    int32_t temperature; // imu control temperature
} temp_cali_t;
// gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; // x,y,z
    fp32 scale[3];  // x,y,z
} imu_cali_t;
//! STEP 2 添加新的校准数据结构体 BEGIN
/*
    >   长度必须是 4 字节倍数
*/

//! STEP 2 添加新的校准数据结构体 END

extern void cali_param_init(void);
extern int8_t get_control_temperature(void);
extern void calibrate_task(void const *pvParameters);

#endif
