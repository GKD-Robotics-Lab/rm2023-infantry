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
  * >                   该函数在参数为 CALI_FUNC_CMD_ON 的情况下返回 1 表示校准完成，否则为未完成
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "can_receive.h"
#include "remote_control.h"
#include "INS_task.h"
#include "gimbal_task.h"

static void RC_cmd_to_calibrate(void);
static void cali_data_read(void);
static void cali_data_write(void);
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   // header device cali function
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   // gyro device cali function
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); // gimbal device cali function

//* 校准设备结构体 -> 该结构体的初始化 cali_param_init() 放在 main.c 中
cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

//! STEP 3 添加新校准设备所用的一系列变量 BEGIN
/*
    >  添加新校准设备名称在 cali_name[CALI_LIST_LENGHT][3]
    >  申明校准数据变量 xxx_cali_t xxx_cail
    >  添加校准数据变量地址在 cali_data_address[CALI_LIST_LENGHT]
    >  在 cali_data_len_word[CALI_LIST_LENGHT] 添加数据长度 (word)
    >  在 cali_hook_fun[CALI_LIST_LENGHT]添加函数
    >  在 "FLASH_WRITE_BUF_LENGHT" 中添加校准数据长度 "sizeof(xxx_cali_t)"
*/

//* 校准设备名称 | name[3]
static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//* 校准数据及其地址 | data
// 设备的校准数据
static head_cali_t head_cali;     // head cali data
static gimbal_cali_t gimbal_cali; // gimbal cali data
static imu_cali_t accel_cali;     // accel cali data
static imu_cali_t gyro_cali;      // gyro cali data
static imu_cali_t mag_cali;       // mag cali data
// cali data address
static uint32_t *cali_data_address[CALI_LIST_LENGHT] = {
    (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
    (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
    (uint32_t *)&mag_cali};

//* 校准数据长度 | data_len_word -> 单位：字
static uint8_t cali_data_len_word[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//* 校准钩子函数 | cali_hook
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

//* 校准数据 flash 设置
// 设备校准数据的总大小
// include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3 + CALI_LIST_LENGHT * 4)
// 设备校准数据 flash 写入缓存区
static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
//! STEP 3 添加新校准设备所用的一系列变量 END

//* other
// remote control point
static const RC_ctrl_t *calibrate_RC;
// 校准计时用 tick
static uint32_t calibrate_systemTick;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

/**
 * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
 * @param[in]      none
 * @retval         none
 *
 * @note           该函数应放在 main.c 中
 */
void cali_param_init(void)
{
    uint8_t i = 0;

    //* 初始化相关数据
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        cali_sensor[i].data_len_word = cali_data_len_word[i];
        cali_sensor[i].data          = cali_data_address[i];
        cali_sensor[i].cali_hook     = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    //* 从 flash 中读取以前的校准值
    cali_data_read();

    //* 如果已经被校准过，执行 init 钩子函数，以使用以前的校准值
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_sensor[i].cali_done == CALIED_FLAG) {
            if (cali_sensor[i].cali_hook != NULL) {
                // if has been calibrated, set to init
                // 如果已经被校准过，设置为 init
                cali_sensor[i].cali_hook(cali_data_address[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

/**
 * @brief          校准任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;

    calibrate_RC = get_remote_ctrl_point_cali();

    while (1) {

        //* 从遥控器获取 (并执行) 校准指令
        //  对于云台和陀螺仪，需要校准时该函数将置位 cali_cmd，其他校准在该函数内部进行
        RC_cmd_to_calibrate();

        //* 检查 cali_cmd，若为 1 则需要执行校准
        for (i = 0; i < CALI_LIST_LENGHT; i++) {
            if (cali_sensor[i].cali_cmd) {
                if (cali_sensor[i].cali_hook != NULL) {

                    if (cali_sensor[i].cali_hook(cali_data_address[i], CALI_FUNC_CMD_ON)) {
                        // done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        // set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        // write
                        cali_data_write();
                    }
                }
            }
        }
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          使用遥控器开始校准
 * @note           对于云台和陀螺仪，需要校准时该函数将置位 cali_cmd 校准会在 calibrate_task 中执行，底盘和温度校准将在该函数内部进行
 * @param[in]      none
 * @retval         none
 */
static void RC_cmd_to_calibrate(void)
{
    static uint8_t rc_action_flag     = 0; // 摇杆行为标志位
    static const uint8_t BEGIN_FLAG   = 1;
    static const uint8_t GIMBAL_FLAG  = 2;
    static const uint8_t GYRO_FLAG    = 3;
    static const uint8_t CHASSIS_FLAG = 4;

    static uint8_t i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time       = 0;
    static uint16_t rc_cmd_time       = 0;

    //* 如果已经在校准，就返回
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_sensor[i].cali_cmd) {
            buzzer_time    = 0;
            rc_cmd_time    = 0;
            rc_action_flag = 0;

            return;
        }
    }

    //* 检查遥控器行为的标志位，判断应该进行什么校准
    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_DURATION) {
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_flag    = BEGIN_FLAG;
        rc_cmd_time       = 0;
    } else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_DURATION) {
        // gimbal cali
        rc_action_flag                    = 0;
        rc_cmd_time                       = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
        cali_buzzer_off();
    } else if (rc_action_flag == GYRO_FLAG && rc_cmd_time > RC_CMD_DURATION) {
        // gyro cali
        rc_action_flag                  = 0;
        rc_cmd_time                     = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        // ? 校准陀螺仪时顺便校准温度？
        // update control temperature
        head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
            head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
        cali_buzzer_off();
    } else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_DURATION) {
        // TODO 底盘重设 ID 也一块放在这了
        rc_action_flag = 0;
        rc_cmd_time    = 0;
        // send CAN reset ID cmd to M3508
        // 发送CAN重设ID命令到3508
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        cali_buzzer_off();
    }

    //* 判断摇杆的行为，设置摇杆行为标志位 rc_action_flag
    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag == 0) {
        // two rockers set to  \../, hold for 2 seconds,
        // 两个摇杆打成 \../,保持2s
        rc_cmd_time++;
    } else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0) {
        // two rockers set '\/', hold for 2 seconds
        // 两个摇杆打成'\/',保持2s
        rc_cmd_time++;
        rc_action_flag = GIMBAL_FLAG;
    } else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0) {
        // two rocker set to ./\., hold for 2 seconds
        // 两个摇杆打成./\.,保持2s
        rc_cmd_time++;
        rc_action_flag = GYRO_FLAG;
    } else if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0) {
        // two rocker set to /''\, hold for 2 seconds
        // 两个摇杆打成/''\,保持2s
        rc_cmd_time++;
        rc_action_flag = CHASSIS_FLAG;
    } else {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    //*
    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME) {
        // over 20 seconds, end
        // 超过20s,停止
        //? 蜂鸣器呢？
        rc_action_flag = 0;
        return;
    } else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_action_flag != 0) {
        //? rc_action_flag != 0 必然有 rc_cmd_systemTick != 0 吧
        rc_cali_buzzer_middle_on();
    } else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_action_flag != 0) {
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0) {
        buzzer_time++;
    }

    if (buzzer_time > RC_CALI_BUZZER_CYCLE_TIME && rc_action_flag != 0) {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0) {
        cali_buzzer_off();
    }
}

/**
 * @brief          get imu control temperature, unit ℃
 * @param[in]      none
 * @retval         imu control temperature
 */
/**
 * @brief          获取imu控制温度, 单位℃
 * @param[in]      none
 * @retval         imu控制温度
 */
int8_t get_control_temperature(void)
{

    return head_cali.temperature;
}

/**
 * @brief          get latitude, default 22.0f
 * @param[out]     latitude: the point to fp32
 * @retval         none
 */
/**
 * @brief          获取纬度,默认22.0f
 * @param[out]     latitude:fp32指针
 * @retval         none
 */
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL) {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG) {
        *latitude = head_cali.latitude;
    } else {
        *latitude = 22.0f;
    }
}

/**
 * @brief          read cali data from flash
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_EX_DATA_LEN_WORD * 4];
    uint8_t i       = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++) {

        // read the data in flash,
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].data, cali_sensor[i].data_len_word);

        offset += cali_sensor[i].data_len_word * 4;

        // read the name and cali flag,
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_EX_DATA_LEN_WORD);

        cali_sensor[i].name[0]   = flash_read_buf[0];
        cali_sensor[i].name[1]   = flash_read_buf[1];
        cali_sensor[i].name[2]   = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        offset += CALI_EX_DATA_LEN_WORD * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL) {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

/**
 * @brief          write the data to flash
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          往flash写入校准数据
 * @param[in]      none
 * @retval         none
 */
static void cali_data_write(void)
{
    uint8_t i       = 0;
    uint16_t offset = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        // copy the data of device calibration data
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].data, cali_sensor[i].data_len_word * 4);
        offset += cali_sensor[i].data_len_word * 4;

        // copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_EX_DATA_LEN_WORD * 4);
        offset += CALI_EX_DATA_LEN_WORD * 4;
    }

    // erase the page
    cali_flash_erase(FLASH_USER_ADDR, 1);
    // write data
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  * @note           id -> 直接设定；
  * @note           陀螺仪的设定温度 temperature -> ADC 温度传感器温度加 6℃；
  * @note           纬度 latitude -> 直接设定
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        //        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    // imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 8;
    // head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

    local_cali_t->firmware_version = FIRMWARE_VERSION;
    // shenzhen latitude
    local_cali_t->latitude = 22.0f;

    return 1;
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  * @note           调用云台校准的钩子函数，未完成将开启蜂鸣器，完成将关闭蜂鸣器
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle)) {
            cali_buzzer_off();

            return 1;
        } else {
            gimbal_start_buzzer();

            return 0;
        }
    }

    return 0;
}

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  * @note           调用 INS_cali_gyro 计算陀螺仪零漂
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        INS_set_cali_gyro(local_cali_t->scale, local_cali_t->offset);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        static uint16_t count_time = 0;
        INS_cali_gyro(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME) {
            count_time = 0;
            cali_buzzer_off();
            gyro_cali_enable_control();
            return 1;
        } else {
            gyro_cali_disable_control(); // disable the remote control to make robot no move
            imu_start_buzzer();

            return 0;
        }
    }

    return 0;
}

//! STEP 4 实现新的校准钩子函数 bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd) BEGIN

//! STEP 4 实现新的校准钩子函数 bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd) END
