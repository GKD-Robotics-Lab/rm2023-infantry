#ifndef AHRS_H
#define AHRS_H

#include "AHRS_middleware.h"

/**
  * @brief          ���ݼ��ٶȵ����ݣ������Ƶ����ݽ�����Ԫ����ʼ��
  * @param[in]      ��Ҫ��ʼ������Ԫ������
  * @param[in]      ���ڳ�ʼ���ļ��ٶȼ�,(x,y,z)��Ϊ�� ��λ m/s2 
  * @param[in]      ���ڳ�ʼ���Ĵ����Ƽ�,(x,y,z)��Ϊ�� ��λ uT
  * @retval         ���ؿ�
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          ���������ǵ����ݣ����ٶȵ����ݣ������Ƶ����ݽ�����Ԫ������
  * @param[in]      ��Ҫ���µ���Ԫ������
  * @param[in]      ���¶�ʱʱ�䣬�̶���ʱ���ã�����1000Hz�����������Ϊ0.001f,
  * @param[in]      ���ڸ��µ�����������,����˳��(x,y,z) ��λ rad
  * @param[in]      ���ڳ�ʼ���ļ��ٶ�����,����˳��(x,y,z) ��λ m/s2 
  * @param[in]      ���ڳ�ʼ���Ĵ���������,����˳��(x,y,z) ��λ uT
  * @retval         1:���³ɹ�, 0:����ʧ��
  */
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ����ƫ��yaw
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���ص�ƫ����yaw ��λ rad
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ���Ǹ����� pitch
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���صĸ����� pitch ��λ rad
  */
extern fp32 get_pitch(const fp32 quat[4]);
/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ���Ǻ���� roll
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���صĺ���� roll ��λ rad
  */
extern fp32 get_roll(const fp32 quat[4]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ����yaw��pitch��roll
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @param[in]      ���ص�ƫ����yaw ��λ rad
  * @param[in]      ���صĸ�����pitch  ��λ rad
  * @param[in]      ���صĺ����roll ��λ rad
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
/**
  * @brief          ���ص�ǰ���������ٶ�
  * @param[in]      ��
  * @retval         �����������ٶ� ��λ m/s2
  */
extern fp32 get_carrier_gravity(void);

#endif
