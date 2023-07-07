/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

// Ħ�����ٶ�
#define FRIC_UP 4850  //?? 1400  ����״̬
#define FRIC_DOWN 4850  //?? 1320  ����״̬
#define FRIC_OFF 0  //?? 1000  ֹͣ״̬

//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    2500.0f	//ԭʼ100.0f

//���Ħ���ּ���� �ر�
//#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//����Ħ����
#define FRIC_MODE_SWITCH_KEYBOARD         KEY_PRESSED_OFFSET_F

//�л����䡢���䰴��
#define SHOOT_MODE_SWITCH_KEYBOARD				KEY_PRESSED_OFFSET_B

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             400  //ԭʼ400		��곤��ʱ��
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              300  //ԭʼ2000		ң��������ʱ��
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//�����ٶ�
#define TRIGGER_SPEED               8.0f  	//ԭʼ10
#define CONTINUE_TRIGGER_SPEED      13.0f  	//ԭʼ13
#define READY_TRIGGER_SPEED         5.0f 		//δʹ��

#define TRIGGER_LONG_TIME						RC_S_LONG_TIME-5			//����: ���ε��䲦���������ʱ��

#define KEY_OFF_JUGUE_TIME          500
// ����͵��Ƿ����: ΢�������Ѱ���,�͵����
#define SWITCH_TRIGGER_ON           0			// �͵����΢������ON: ���0���ɽ��ô˹��ܣ������ŵ�ѹһֱΪ0�����޷��Ӹ����Ͻ���
// ����͵��Ƿ����: ΢������δ����,�͵�δ���
#define SWITCH_TRIGGER_OFF          1			// �͵����΢������OFF

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     40

typedef enum
{
    SHOOT_STOP = 0,						// ״̬<Ħ����ֹͣ>
    SHOOT_READY_FRIC,  				// ״̬<Ħ���ֿ���>
    SHOOT_READY_BULLET,				// ״̬<��ʼ�͵�>
    SHOOT_READY,							// ״̬<�������>
    SHOOT_BULLET,							// ״̬<��ʼ���-����>
    SHOOT_CONTINUE_BULLET,		// ״̬<��ʼ���-����>
    SHOOT_DONE,								// ״̬<������>
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
		
		/* 2021.1.15��� ���ڽ���CV����*/
		const can_CV_t *can_CV;
		/********************************/
} shoot_control_t;

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t shoot_control_loop(gimbal_control_t *gimbal_shoot);

#endif
