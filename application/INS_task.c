/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================
  * > 在该版本的代码中为了节约 CPU 的时间没有使用磁力计的数据
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "config.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "AHRS.h"

#ifdef IMU_DATA_PRINT
#include "bsp_usart.h"
#endif

#include "calibrate_task.h"
#include "detect_task.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}
#define IST8310_BOARD_INSTALL_SPIN_MATRIX {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}

static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void imu_temp_control(fp32 temp);
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

//* dma buffer
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

//* 用于 SPI 通信调度的 flag
volatile uint8_t gyro_update_flag       = 0;
volatile uint8_t accel_update_flag      = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag        = 0;
volatile uint8_t imu_start_dma_flag     = 0;

//* 陀螺仪数据
bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

//* 磁力计数据
ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

//* 温度数据及其 PID
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_typedef imu_temp_pid;

static const float timing_time = 0.001f; // tast run time , unit s.任务运行的时间 单位 s

//* 加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
// TODO 低通滤波系数的设置
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

//* 根据安装方式旋转和计算零漂后得到的传感器数据
static fp32 INS_gyro[3]  = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3]   = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3]        = {0.0f, 0.0f, 0.0f};

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void INS_task(void const *pvParameters)
{
    //* 加速度计及磁力计初始化
    // wait a time
    osDelay(INS_TASK_INIT_TIME);
    while (BMI088_init()) {
        osDelay(100);
    }
    while (ist8310_init()) {
        osDelay(100);
    }

    //* 更新加速度计值
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    // rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    //* 温度传感器初始化
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT, TEMPERATURE_PID_DEAD_BAND);
    //* 四元数姿态解算初始化
    AHRS_init(INS_quat, INS_accel, INS_mag);
    //* 初始化滤波器的记录值
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

    //* 获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //* SPI 初始化
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    while (1) {
        //* 传感器数值更新
        // wait spi DMA tansmit done
        // 等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }

        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        // rotate and zero drift
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        // 加速度计低通滤波
        // accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        //* 姿态解算
        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_ANGLE_YAW_ADDRESS_OFFSET, INS_angle + INS_ANGLE_PITCH_ADDRESS_OFFSET, INS_angle + INS_ANGLE_ROLL_ADDRESS_OFFSET);

#ifdef IMU_DATA_PRINT
        usart1_printf("%f, %f, %f, %f, %f, %f\r\n", *(INS_angle + INS_ANGLE_ROLL_ADDRESS_OFFSET), *(INS_angle + INS_ANGLE_PITCH_ADDRESS_OFFSET), *(INS_angle + INS_ANGLE_YAW_ADDRESS_OFFSET), INS_gyro[0], INS_gyro[1], INS_gyro[2]);
#endif

        // because no use ist8310 and save time, no use
        if (mag_update_flag &= 1 << IMU_DR_SHFITS) {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            // ist8310_read_mag(ist8310_real_data.mag);
        }
    }
}

/**
 * @brief          根据安装方式旋转和零漂计算更新传感器数据
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++) {
        gyro[i]  = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i]   = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate) {
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        if (imu_temp_pid.out < 0.0f) {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    } else {
        // 在没有达到设置的温度，一直最大功率加热
        // in beginning, max power
        if (temp > get_control_temperature()) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                // 达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate   = 1;
                imu_temp_pid.Iout = IMU_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(IMU_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief          校准陀螺仪
 * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
 * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
 * @retval         none
 */
void INS_cali_gyro_hook(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    //* 开始校准时将原数据归零
    if (*time_count == 0) {
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;
    }

    //* 迭代校准值
    gyro_offset[0] = gyro_offset[0] - 0.0003f * INS_gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * INS_gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * INS_gyro[2];
    (*time_count)++;

    //* 校准值错误判断
    if (ABS(gyro_offset[0]) > GYRO_OFFSET_MAX) gyro_offset[0] = 0;
    if (ABS(gyro_offset[1]) > GYRO_OFFSET_MAX) gyro_offset[1] = 0;
    if (ABS(gyro_offset[2]) > GYRO_OFFSET_MAX) gyro_offset[2] = 0;

    //* 保存校准值
    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
    cali_scale[0]  = 1.0f;
    cali_scale[1]  = 1.0f;
    cali_scale[2]  = 1.0f;
}

/**
 * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂
 * @retval         none
 */
void INS_set_cali_gyro_hook(fp32 cali_scale[3], fp32 cali_offset[3])
{
    //* 校准值错误判断
    if (ABS(cali_offset[0]) > GYRO_OFFSET_MAX) cali_offset[0] = 0;
    if (ABS(cali_offset[1]) > GYRO_OFFSET_MAX) cali_offset[1] = 0;
    if (ABS(cali_offset[2]) > GYRO_OFFSET_MAX) cali_offset[2] = 0;

    gyro_offset[0] = cali_offset[0];
    gyro_offset[1] = cali_offset[1];
    gyro_offset[2] = cali_offset[2];
}

/**
 * @brief          获取四元数
 * @param[in]      none
 * @retval         INS_quat的指针
 */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
 * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
 * @param[in]      none
 * @retval         INS_angle的指针
 */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
 * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
 * @param[in]      none
 * @retval         INS_gyro的指针
 */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
 * @param[in]      none
 * @retval         INS_accel的指针
 */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
 * @param[in]      none
 * @retval         INS_mag的指针
 */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == DRDY_IST8310_Pin) {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    } else if (GPIO_Pin == GPIO_PIN_0) {

        // wake up the task
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // 开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMA2_Stream2_IRQHandler(void)
{

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // gyro read over
        // 陀螺仪读取完毕
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // accel read over
        // 加速度计读取完毕
        if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // temperature read over
        // 温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
