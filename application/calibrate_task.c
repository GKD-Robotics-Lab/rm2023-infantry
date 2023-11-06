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

#include "calibrate_task.h"

#include <string.h>
#include "cmsis_os.h"
#include "main.h"
#include "config.h"
#include "user_lib.h"

#include "bsp_adc.h"
#include "bsp_flash.h"
#ifdef CALI_BUZZER_ON
#include "bsp_buzzer.h"
#endif
#ifdef CALI_USART_PRINT
#include "bsp_usart.h"
#endif

#include "remote_control.h"
#include "INS_task.h"
#include "detect_task.h"

//* 为精简代码将遥控器相关按键使用宏替代
// 遥控器两个拨杆均打到下
#define is_all_switch_down (switch_is_down(cali_RC->rc.s[0]) && switch_is_down(cali_RC->rc.s[1]))
// 遥控器的摇杆内拨
#define is_rocker_dialed_inward (cali_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && cali_RC->rc.ch[2] > RC_CALI_VALUE_HOLE)
// 遥控器的摇杆外拨
#define is_rocker_dialed_outward (cali_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && cali_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE)
// 遥控器的摇杆都打到下
#define is_all_rocker_down (cali_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && cali_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE)

//* 为精简代码将蜂鸣器行为使用宏替代
// 校准所有设备时蜂鸣器响 3 声
#define cali_all_buzzer          \
    {                            \
        cali_all_start_buzzer(); \
        vTaskDelay(100);         \
        cali_buzzer_off();       \
        vTaskDelay(50);          \
        cali_all_start_buzzer(); \
        vTaskDelay(100);         \
        cali_buzzer_off();       \
        vTaskDelay(50);          \
        cali_all_start_buzzer(); \
        vTaskDelay(300);         \
        cali_buzzer_off();       \
        vTaskDelay(50);          \
    }
// 开始校准设备选择时蜂鸣器响 3 声
#define cali_RC_choose_begin_buzzer          \
    {                                        \
        cali_RC_choose_begin_start_buzzer(); \
        vTaskDelay(150);                     \
        cali_buzzer_off();                   \
        vTaskDelay(75);                      \
        cali_RC_choose_begin_start_buzzer(); \
        vTaskDelay(150);                     \
        cali_buzzer_off();                   \
        vTaskDelay(75);                      \
        cali_RC_choose_begin_start_buzzer(); \
        vTaskDelay(300);                     \
        cali_buzzer_off();                   \
        vTaskDelay(50);                      \
    }
// 校准选择完成时蜂鸣器响声
#define cali_RC_choose_OK_buzzer_loop     \
    {                                     \
        cali_RC_choose_OK_start_buzzer(); \
        vTaskDelay(200);                  \
        cali_buzzer_off();                \
        vTaskDelay(75);                   \
    }

static void RC_cmd_to_calibrate(void);
static uint8_t key_cmd_to_calibrate(uint8_t cmd);
static void calibrate_all(void);
static void cali_data_read(void);
static void cali_data_write(void);
static bool_t cali_temp_hook(uint32_t *cali, bool_t cmd); // temperature cali function
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd); // gyro device cali function

//* 校准设备结构体 -> 该结构体的初始化 cali_param_init() 放在 main.c 中
cali_device_t cali_device[CALI_LIST_LENGHT];

//! STEP 3 添加新校准设备所用的一系列变量 BEGIN
/*
    >  添加新校准设备名称在 cali_name[CALI_LIST_LENGHT][3]
    >  申明校准数据变量 xxx_cali_t xxx_cail
    >  添加校准数据变量地址在 cali_data_address[CALI_LIST_LENGHT]
    >  在 cali_data_len_word[CALI_LIST_LENGHT] 添加数据长度 (word)
    >  在 cali_hook_fun[CALI_LIST_LENGHT]添加函数
    >  在 "FLASH_WRITE_BUF_LENGHT" 中添加校准数据长度 "sizeof(xxx_cali_t)"
    >  注意顺序要一一对应，这里的顺序也是校准的优先级
*/

//* 校准设备名称 | name[3]
static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"TMP", "GYR", "ACC", "MAG"};

//* 校准数据及其地址 | data
// 设备的校准数据
static temp_cali_t temp_cali; // head cali data
static imu_cali_t gyro_cali;  // gyro cali data
static imu_cali_t accel_cali; // accel cali data
static imu_cali_t mag_cali;   // mag cali data
// cali data address
static uint32_t *cali_data_address[CALI_LIST_LENGHT] = {(uint32_t *)&temp_cali,
                                                        (uint32_t *)&gyro_cali,
                                                        (uint32_t *)&accel_cali,
                                                        (uint32_t *)&mag_cali};

//* 校准数据长度 | data_len_word -> 单位：字
static uint8_t cali_data_len_word[CALI_LIST_LENGHT] = {sizeof(temp_cali_t) / 4,
                                                       sizeof(imu_cali_t) / 4,
                                                       sizeof(imu_cali_t) / 4,
                                                       sizeof(imu_cali_t) / 4};

//* 校准钩子函数 | cali_hook
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_temp_hook, cali_gyro_hook, NULL, NULL};

//* 校准数据 flash 设置
// 设备校准数据的总大小
// include temp,gyro,accel,mag. gyro,accel and mag have the same data struct. total 4(CALI_LIST_LENGHT) devices, need data lenght + 4 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT (sizeof(temp_cali_t) + sizeof(imu_cali_t) * 3 + CALI_LIST_LENGHT * 4)
// 设备校准数据 flash 写入缓存区
static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
//! STEP 3 添加新校准设备所用的一系列变量 END

//* RC
// remote control point
static const RC_ctrl_t *cali_RC;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

/**
 * @brief          校准初始化函数
 *
 * @note           该函数应放在 main.c 中
 */
void cali_param_init(void)
{
    uint8_t i = 0;

    //* 初始化相关数据
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        cali_device[i].data_len_word = cali_data_len_word[i];
        cali_device[i].data          = cali_data_address[i];
        cali_device[i].cali_hook     = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    //* 从 flash 中读取以前的校准值
    cali_data_read();

    //* 如果已经被校准过，执行 init 钩子函数，以使用以前的校准值
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_device[i].cali_done == CALIED_FLAG) {
            if (cali_device[i].cali_hook != NULL) {
                // if has been calibrated, set to init
                // 如果已经被校准过，设置通过 init 模式下的钩子函数设置校准值
                // hook 函数返回 1 表示每次上电都需要校准
                cali_device[i].cali_cmd = cali_device[i].cali_hook(cali_data_address[i], CALI_FUNC_CMD_INIT);
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

    cali_RC = get_remote_control_point();

    // 要在 detect 任务初始化完成后再检测遥控器
    vTaskDelay(CALIBRATE_CONTROL_INIT_TIME);

    //* 遥控器刚连接时如果拨杆全部打到下且摇杆都打到下，则对所有设备进行校准
    while (toe_is_error(DBUS_TOE)) {
        // 遥控器未连接时也可以使用按键
        if (key_cmd_to_calibrate(KEY_SET_CALI_DONT_CHECK) == 1)
            break;
        vTaskDelay(CALIBRATE_CONTROL_TIME);
    }
    if (is_all_switch_down && is_all_rocker_down)
        calibrate_all();

    while (1) {
        //* 从遥控器获取 (并执行) 校准指令
        RC_cmd_to_calibrate();
        //* 从按键获取校准指令
        key_cmd_to_calibrate(KEY_SET_CALI_CHECK);

        //* 检查 cali_cmd，若为 1 则需要执行校准
        for (i = 0; i < CALI_LIST_LENGHT; i++) {
            if (cali_device[i].cali_cmd) {
                if (cali_device[i].cali_hook != NULL) {
                    if (cali_device[i].cali_hook(cali_data_address[i], CALI_FUNC_CMD_ON)) {
                        // done
                        cali_device[i].name[0] = cali_name[i][0];
                        cali_device[i].name[1] = cali_name[i][1];
                        cali_device[i].name[2] = cali_name[i][2];
                        // set 0x55
                        cali_device[i].cali_done = CALIED_FLAG;

                        cali_device[i].cali_cmd = 0;
                        // write
                        cali_data_write();
                    }
                } else {
                    // cali_hook 为 NULL 说明校准还没有实现，直接校准完成
                    cali_device[i].name[0]   = cali_name[i][0];
                    cali_device[i].name[1]   = cali_name[i][1];
                    cali_device[i].name[2]   = cali_name[i][2];
                    cali_device[i].cali_done = CALIED_FLAG;
                    cali_device[i].cali_cmd  = 0;
                    cali_data_write();
                }
                break; // 同一时刻只能进行一个设备的校准
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

    static int8_t rc_flag = RC_FLAG_NO_CMD; // 摇杆行为标志位

    static int32_t rc_cmd_time = 0;

    //* 如果已经在校准，就返回
    for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++) {
        if (cali_device[i].cali_cmd) {
            rc_flag = RC_FLAG_NO_CMD;

            return;
        }
    }

    //* 判断摇杆的行为进行计时自增
    if (!is_all_switch_down) {
        // 拨杆没有都拨到下，直接重置标志位并返回
        rc_cmd_time = 0;
        rc_flag     = RC_FLAG_NO_CMD;

        return;
    } else {
        if (rc_flag == RC_FLAG_NO_CMD && is_rocker_dialed_inward) {
            if (rc_cmd_time < 0)
                rc_cmd_time = 0;
            else
                rc_cmd_time++;
        } else if (rc_flag > RC_FLAG_NO_CMD && is_rocker_dialed_outward) {
            if (rc_cmd_time < 0)
                rc_cmd_time = 0;
            else
                rc_cmd_time++;
        } else if (rc_flag > RC_FLAG_NO_CMD && rc_cmd_time >= RC_CMD_VERIFY_TIME && !is_rocker_dialed_inward) {
            rc_cmd_time = 0;
            rc_flag++;
        } else {
            rc_cmd_time--;
        }
    }

#ifdef BUZZER_ON
    // 一次选择达到时长后，打开蜂鸣器
    if (rc_flag > RC_FLAG_NO_CMD) {
        if (rc_cmd_time >= RC_CMD_VERIFY_TIME)
            cali_RC_choose_verified_start_buzzer();
        else
            cali_buzzer_off();
    }
#endif

    if (rc_flag == RC_FLAG_NO_CMD && rc_cmd_time >= RC_CMD_START_TIME) {
        rc_flag     = RC_FLAG_BEGIN;
        rc_cmd_time = 0;
#ifdef BUZZER_ON
        // 进入校准项目选择，响三下蜂鸣器
        cali_RC_choose_begin_buzzer;
#endif
    } else if (rc_flag > RC_FLAG_NO_CMD && rc_cmd_time <= -RC_CMD_STOP_TIME) {
        //* 检查遥控器行为的标志位，判断应该进行什么校准
        switch (rc_flag) {
            case RC_FLAG_GYRO:
                cali_device[CALI_GYRO].cali_cmd = 1;
                break;
            default:
                break;
        }
#ifdef CALI_USART_PRINT
        switch (rc_flag) {
            case RC_FLAG_GYRO:
                usart1_printf("=============!!! RC -> gyro calibrate !!!==============\r\n\r\n");
                break;
            default:
                break;
        }
#endif

#ifdef BUZZER_ON
        // 选到了第几个项目，就对应响几次
        for (uint8_t i = 0; i < rc_flag; i++) {
            cali_RC_choose_OK_buzzer_loop;
        }
#endif

        rc_flag     = RC_FLAG_NO_CMD;
        rc_cmd_time = 0;
    }
}

/**
 * @brief 按键持续按下一秒则全部校准
 *
 * @param cmd KEY_SET_CALI_CHECK 检查是否有在校准的；KEY_SET_CALI_DONT_CHECK 不检查
 * @return uint8_t
 */
static uint8_t key_cmd_to_calibrate(uint8_t cmd)
{
    static uint32_t key_press_time = 0;

    if (cmd == KEY_SET_CALI_CHECK) {
        //* 如果已经在校准，就返回 (但如果在等待遥控器连接时，不进行检查)
        for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++) {
            if (cali_device[i].cali_cmd) {
                key_press_time = 0;
                return 0;
            }
        }
    }

    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        key_press_time++;
    else
        key_press_time = 0;

    if (key_press_time >= KEY_LONG_TIME) {
        calibrate_all();
        return 1;
    }

    return 0;
}

/**
 * @brief 将所有校准设备的 cmd 设为 1
 *
 */
static void calibrate_all(void)
{
    //* 使能所有的校准任务
    for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++)
        cali_device[i].cali_cmd = 1;

#ifdef CALI_USART_PRINT
    //* 串口打印校准所有的信息
    usart1_printf("=================!!! calibrate all !!!=================\r\n\r\n");
#endif

#ifdef CALI_BUZZER_ON
    //* 蜂鸣器响 3 声
    cali_all_buzzer;
#endif
}

/**
 * @brief          获取imu控制温度, 单位℃
 * @param[in]      none
 * @retval         imu控制温度
 */
int8_t get_control_temperature(void)
{
    return temp_cali.temperature;
}

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
        cali_flash_read(FLASH_USER_ADDR + offset, cali_device[i].data, cali_device[i].data_len_word);

        offset += cali_device[i].data_len_word * 4;

        // read the name and cali flag,
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_EX_DATA_LEN_WORD);

        cali_device[i].name[0]   = flash_read_buf[0];
        cali_device[i].name[1]   = flash_read_buf[1];
        cali_device[i].name[2]   = flash_read_buf[2];
        cali_device[i].cali_done = flash_read_buf[3];

        offset += CALI_EX_DATA_LEN_WORD * 4;

        if (cali_device[i].cali_done != CALIED_FLAG && cali_device[i].cali_hook != NULL) {
            cali_device[i].cali_cmd = 1; // 没有读到数据要在任务中进行校准
        }
    }
}

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
        memcpy((void *)(flash_write_buf + offset), (void *)cali_device[i].data, cali_device[i].data_len_word * 4);
        offset += cali_device[i].data_len_word * 4;

        // copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buf + offset), (void *)cali_device[i].name, CALI_EX_DATA_LEN_WORD * 4);
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
  * @note           陀螺仪的设定温度 temperature -> ADC 温度传感器温度加 10℃；
  * @note           纬度 latitude -> 直接设定
  */
static bool_t cali_temp_hook(uint32_t *cali, bool_t cmd)
{
    temp_cali_t *local_cali = (temp_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        return 1; // 每次上电都要重新校准
    }

#ifdef CALI_USART_PRINT
    //* 串口打印校准所有的信息
    usart1_printf("-----------!! temperature calibrate start !!-----------\r\n");
#endif

    // imu control temperature
    local_cali->temperature = (int8_t)(bsp_adc_get_temperature()) + 10;
    if (local_cali->temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
        local_cali->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

#ifdef CALI_USART_PRINT
    //* 串口打印校准所有的信息
    usart1_printf("imu temperature control value set : %d\r\n", get_control_temperature());
    usart1_printf("---------!! temperature calibrate complete !!----------\r\n\r\n");
#endif

    return 1;
}

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  * @note           调用 INS_cali_gyro_hook 计算陀螺仪零漂
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali     = (imu_cali_t *)cali;
    static uint16_t count_time = 0;

    //* 写入 flash 中的数据
    if (cmd == CALI_FUNC_CMD_INIT) {
        INS_set_cali_gyro_hook(local_cali->scale, local_cali->offset);
        return 0;
    }

    //* 陀螺仪零漂迭代计算钩子函数
    INS_cali_gyro_hook(local_cali->scale, local_cali->offset, &count_time);

#ifdef CALI_USART_PRINT
    if (count_time == 0)
        usart1_printf("--------------!! gyro calibrate start !!---------------\r\n");
#endif

#ifdef CALI_USART_GYRO_DATA_PRINT
    usart1_printf("%f, %f, %f\r\n", local_cali->offset[0], local_cali->offset[1], local_cali->offset[2]);
#endif

    //* 20s 后结束校准
    if (count_time > GYRO_CALIBRATE_TIME) {
        count_time = 0;
        cali_enable_RC_control();
#ifdef CALI_GYRO_BUZZER_ON
        cali_buzzer_off();
#endif
#ifdef CALI_USART_PRINT
        usart1_printf("------------!! gyro calibrate compelete !!-------------\r\n\r\n");
#endif
        return 1;
    } else {
        cali_disable_RC_control(); // disable the remote control to make robot no move
#ifdef CALI_GYRO_BUZZER_ON
        cali_imu_start_buzzer();
#endif
        return 0;
    }

    return 0;
}

//! STEP 4 实现新的校准钩子函数 bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd) BEGIN
/*
    >这里 hook 函数的作用是 1.启动校准 2.将得到的校准值拿到设备那边用
    >                   cmd 参数有两个：
    >                       CALI_FUNC_CMD_INIT，已经校准过，直接使用校准值
    >                       CALI_FUNC_CMD_ON，开始校准
    >                   该函数在参数为 CALI_FUNC_CMD_INIT 的情况下返回 1 表示每次上电都需要校准，否则为不需要
    >                   该函数在参数为 CALI_FUNC_CMD_ON 的情况下返回 1 表示校准完成，否则为未完成
    >                   这些函数对应设置的校准值必须是在任务启动前就已经存在的静态变量
*/

//! STEP 4 实现新的校准钩子函数 bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd) END
