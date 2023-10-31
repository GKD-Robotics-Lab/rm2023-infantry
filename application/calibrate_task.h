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
  * >           使用遥控器进行开始校准
  * >               第一步:遥控器的两个开关都打到下
  * >               第二步:两个摇杆打成\../,保存两秒.\.代表左摇杆向右下打.
  * >               第三步:摇杆打成./\. 开始陀螺仪校准
  * >                      或者摇杆打成'\/' 开始云台校准
  * >                      或者摇杆打成/''\ 开始底盘校准
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
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

#define CALIBRATE_CONTROL_TIME 1 // osDelay time,  means 1ms.1ms 系统延时

//* calibrate
// cali_done
#define CALIED_FLAG 0x55 // means it has been calibrated
// cali func cmd
#define CALI_FUNC_CMD_ON   1 // need calibrate,设置校准
#define CALI_FUNC_CMD_INIT 0 // has been calibrated, set value to init.已经校准过，设置校准值
// cali param
#define CALI_EX_DATA_LEN_WORD 1 // 额外数据 name[3] 和 cali_done 的长度 (word)
// gryo : cali time
#define GYRO_CALIBRATE_TIME 20000 // gyro calibrate time,陀螺仪校准时间
// head
#define SELF_ID          0     // ID
#define FIRMWARE_VERSION 12345 // handware version.

//* buzzer
// when imu is calibrating ,buzzer set frequency and strength. 当imu在校准,蜂鸣器的设置频率和强度
#define imu_start_buzzer() buzzer_on(95, 10000)
// when gimbal is calibrating ,buzzer set frequency and strength.当云台在校准,蜂鸣器的设置频率和强度
#define gimbal_start_buzzer()      buzzer_on(31, 19999)
#define cali_buzzer_off()          buzzer_off() // buzzer off，关闭蜂鸣器
#define rc_cali_buzzer_middle_on() gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()  imu_start_buzzer()

//* temperature
// get stm32 chip temperature, to calc imu control temperature.获取stm32片内温度，计算imu的控制温度
#define cali_get_mcu_temperature() get_temprate()
#define GYRO_CONST_MAX_TEMP        45.0f // max control temperature of gyro,最大陀螺仪控制温度

//* flash
#define FLASH_USER_ADDR                     ADDR_FLASH_SECTOR_9                                 // write flash page 9,保存的flash页地址
#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                 // flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) // flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))          // flash erase function,flash擦除函数

//* RC
// RC point
#define get_remote_ctrl_point_cali() get_remote_control_point() // get the remote control point，获取遥控器指针
// RC control
#define gyro_cali_disable_control() RC_unable() // when imu is calibrating, disable the remote control.当imu在校准时候,失能遥控器
#define gyro_cali_enable_control()  RC_restart(SBUS_RX_BUF_NUM)
// RC value limit
#define RC_CALI_VALUE_HOLE 600 // remote control threshold, the max value of remote control channel is 660.
// RC time set
#define CALIBRATE_END_TIME         20000 // you have 20 seconds to calibrate by remote control. 有20s可以用遥控器进行校准
#define RC_CMD_DURATION            2000  // 遥控器控制持续时间
#define RC_CALI_BUZZER_CYCLE_TIME  400
#define RC_CALI_BUZZER_PAUSE_TIME  200
#define RC_CALI_BUZZER_START_TIME  0     // in the beginning, buzzer frequency change to low frequency of imu calibration.当开始校准的时候,蜂鸣器切成低频声音
#define RC_CALI_BUZZER_MIDDLE_TIME 10000 // when 10 second, buzzer frequency change to high frequency of gimbal calibration.当10s的时候,蜂鸣器切成高频声音

//* cali device structure
typedef struct
{
    uint8_t name[3];                                  // device name
    uint8_t cali_done;                                // 0x55 means has been calibrated
    uint8_t data_len_word : 7;                        // buf lenght
    uint8_t cali_cmd : 1;                             // 1 means to run cali hook function,
    uint32_t *data;                                   // link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd); // cali function
} __packed cali_sensor_t;

//* cali device id
//// 其实这个 id 只是用来计算设备列表长度
typedef enum {
    // TODO 写一下校准的目标/目的
    CALI_HEAD   = 0, // 包括 id，陀螺仪的设定温度 temperature，纬度 latitude
    CALI_GIMBAL = 1, // 云台中值校准
    CALI_GYRO   = 2, // 陀螺仪零漂校准
    CALI_ACC    = 3,
    CALI_MAG    = 4,
    //! STEP 1 添加设备名 BEGIN

    //! STEP 1 添加设备名 END
    CALI_LIST_LENGHT,
} cali_id_e;

//* cali data structure
// header device
typedef struct
{
    uint8_t self_id;           // the "SELF_ID"
    uint16_t firmware_version; // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature; // imu control temperature
    fp32 latitude;      // latitude
} head_cali_t;
// gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
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
extern void get_flash_latitude(float *latitude);
extern void calibrate_task(void const *pvParameters);

#endif
