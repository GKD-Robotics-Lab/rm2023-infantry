/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "UI.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off()    fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

/**
  * @brief          ��ʼ��"gimbal_CV"����
  * @param[out]     init: ������init_CV
  * @retval         none
  */
static void CV_init(shoot_control_t *init);


shoot_control_t shoot_control;          //�������


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //��������
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		
		CV_init(&shoot_control);
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(gimbal_control_t *gimbal_shoot)
{
		//״̬�����������жϲ�����״̬��shoot_control_loop���ڸ���״̬���п��Ʋ���
    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)	// ״̬<��ʼ�͵�> (�ٲ��߼�)
    {
				/* �ٷ� */
//        if(shoot_control.key == SWITCH_TRIGGER_OFF)  // �����͵�
//        {
//            //���ò����ֵĲ����ٶ�,��������ת��ת����
//            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
//            trigger_motor_turn_back();
//        }
//        else																				// ֹͣ�͵�
//        {
//            shoot_control.trigger_speed_set = 0.0f;
//            shoot_control.speed_set = 0.0f;
//        }
//        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
//        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
				/* �޸� */
				// ����ж���ֻ�����һ�Σ�֮��ͽ���SHOOT_READY
				shoot_control.trigger_speed_set = 0.0f;
				shoot_control.speed_set = 0.0f;	
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
				shoot_control.shoot_mode = SHOOT_READY;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)  // ״̬<�������>
    {		//�ȴ�����SHOOT_BULLET״̬��ָ��
        //���ò����ֵ��ٶ�
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)  // ״̬<��ʼ���-����>
    {
				/* �ٷ� */
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
				/* �޸� */
//				static uint16_t i=0;
//        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
//        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
//				shoot_control.trigger_speed_set = TRIGGER_SPEED;
//        trigger_motor_turn_back();
//				if(++i > TRIGGER_LONG_TIME)  //���ε��䲦���������ʱ��
//				{
//						i = 0;
//						shoot_control.shoot_mode = SHOOT_DONE;
//				}
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)  // ״̬<��ʼ���-����>
    {
        //���ò����ֵĲ����ٶ�,��������ת��ת����
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        shoot_laser_on(); //���⿪��
        //���㲦���ֵ��PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
		/* ����������ԭ�е�Ħ�����ٶȿ����߼���������������֣��˴�ֻ�����������ѭ����������� */
//    shoot_fric1_on(shoot_control.fric_pwm1);
//    shoot_fric2_on(shoot_control.fric_pwm2);
		gimbal_shoot->fric1_motor.speed_set = -shoot_control.fric_pwm1;
		gimbal_shoot->fric2_motor.speed_set = shoot_control.fric_pwm2;
		PID_calc(&gimbal_shoot->fric1_motor_pid, gimbal_shoot->fric1_motor.speed, gimbal_shoot->fric1_motor.speed_set);
		gimbal_shoot->fric1_motor.give_current = (int16_t)(gimbal_shoot->fric1_motor_pid.out);
		PID_calc(&gimbal_shoot->fric2_motor_pid, gimbal_shoot->fric2_motor.speed, gimbal_shoot->fric2_motor.speed_set);
		gimbal_shoot->fric2_motor.give_current = (int16_t)(gimbal_shoot->fric2_motor_pid.out);
    return shoot_control.given_current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
	  static int16_t last_key_mode = 0;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;  // ״̬<Ħ���ֿ���>
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;  // ״̬<Ħ����ֹͣ>
    }
		
		if ((shoot_control.shoot_rc->key.v & FRIC_MODE_SWITCH_KEYBOARD) && last_key_mode == 0)
    {
				if(shoot_control.shoot_mode == SHOOT_STOP)
				{
					  shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}
				else if(shoot_control.shoot_mode != SHOOT_STOP)
				{
						shoot_control.shoot_mode = SHOOT_STOP;
				}
    }
		UI_set_fric(shoot_control.shoot_mode != SHOOT_STOP);
		last_key_mode = (shoot_control.shoot_rc->key.v & FRIC_MODE_SWITCH_KEYBOARD);
//    //�����е��� ����ʹ�ü��̿���Ħ���� // ֻ��ͨ�����̲��ܽ�������ж���
//    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
//    { 	
//        shoot_control.shoot_mode = SHOOT_READY_FRIC;  // ״̬<Ħ���ֿ���>
//    }
//    //�����е��� ����ʹ�ü��̹ر�Ħ���� // ֻ��ͨ�����̲��ܽ�������ж���
//    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
//    { 	
//        shoot_control.shoot_mode = SHOOT_STOP;  // ״̬<Ħ����ֹͣ>
//    }
		
		// �л����������
		static int16_t last_mode_key = 0;
		static int8_t shoot_mode = 1;
		if(!last_mode_key && (shoot_control.shoot_rc->key.v & SHOOT_MODE_SWITCH_KEYBOARD))
		{
			shoot_mode = !shoot_mode;
		}
		last_mode_key = shoot_control.shoot_rc->key.v & SHOOT_MODE_SWITCH_KEYBOARD;
		
		// ����Ħ���ֵ����ת���Ѵﵽ�趨ֵ(Ħ����ת���ǻ������ӵ�)
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;  // ״̬<��ʼ�͵�> (�ٲ�����)
    }
		// ����͵��Ƿ����: ΢�������Ѱ���,�͵����
    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)  
    {
        shoot_control.shoot_mode = SHOOT_READY;  // ״̬<�������> �͵���ɣ�׼�������
    }
		// ����͵��Ƿ����: ΢������δ����,�͵�δ���
    else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)  
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;  // ״̬<��ʼ�͵�> (�ٲ�����)
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)  // ״̬<�������>
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;  // ״̬<��ʼ���>
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
				/* �ٷ� */
//        if(shoot_control.key == SWITCH_TRIGGER_OFF)
//        {
//            shoot_control.key_time++;
//            if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
//            {
//                shoot_control.key_time = 0;
//                shoot_control.shoot_mode = SHOOT_READY_BULLET;
//            }
//        }
//        else
//        {
//            shoot_control.key_time = 0;
//            shoot_control.shoot_mode = SHOOT_BULLET;
//        }
				/* ���� */
				shoot_control.key_time++;
				if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
				{
						shoot_control.key_time = 0;
						shoot_control.shoot_mode = SHOOT_READY_BULLET;
				}
    }
    
		//����������״̬
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC)  // > SHOOT_READY_FRIC: SHOOT_READY_BULLET \ SHOOT_READY \ SHOOT_BULLET \ SHOOT_CONTINUE_BULLET \ SHOOT_DONE
    {
        //��곤��һֱ�������״̬ ��������
//        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
//        {
//            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//        }
//        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//        {
//            shoot_control.shoot_mode = SHOOT_READY_BULLET;
//        }
				
        /* �����������л�����������䣬ң����һֱ���� */
        if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) || (shoot_mode && shoot_control.press_l))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }
		
		//���ǹ����������ǹ�ڹ��ȣ���ֹͣ���
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > 280))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
		
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //΢������
//    shoot_control.key = BUTTEN_TRIG_PIN;
		shoot_control.key = SWITCH_TRIGGER_ON;  // ����΢������һֱ���ڰ���״̬�������͵����
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
		//�����Ӿ�����������߼�����ʹ���Ӿ�ָ��ģ��switch down���Ӷ��������
    if (shoot_control.shoot_mode != SHOOT_STOP && 
			 (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) || (shoot_control.can_CV->cv_shoot_status == CV_SHOOT_START && shoot_control.can_CV->cv_reco_status == CV_RECO_START)))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

//    //����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
//    static uint16_t up_time = 0;
//    if (shoot_control.press_r)
//    {
//        up_time = UP_ADD_TIME;
//    }
//    if (up_time > 0)
//    {
//        shoot_control.fric1_ramp.max_value = FRIC_UP;
//        shoot_control.fric2_ramp.max_value = FRIC_UP;
//        up_time--;
//    }
//    else
//    {
//        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
//        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
//    }
		/* ��������Ҽ������������ */
		shoot_control.fric1_ramp.max_value = FRIC_DOWN;
		shoot_control.fric2_ramp.max_value = FRIC_DOWN;

}

static void trigger_motor_turn_back(void)  // �����ת
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η��� (���һ���ӵ�)
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
		//ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_FOUR*0.95f);  // ��PI_TEN��ΪPI_FOUR
        shoot_control.move_flag = 1;
    }
//    if(shoot_control.key == SWITCH_TRIGGER_OFF)  // ΢�������ɿ���������һ�����辭����ֱ��º��ɿ�һ��΢������ (�ٲ��߼�)
//    {
//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
    //����Ƕ��ж�
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)  // ����=0.05
    {
        //�趨�Ƕ�û���һֱ��ת
        shoot_control.trigger_speed_set = TRIGGER_SPEED;  // ������ת�ٶ�
        trigger_motor_turn_back();  // ��ת
    }
    else
    {
        shoot_control.move_flag = 0;
				/* ���� */
				shoot_control.shoot_mode = SHOOT_DONE;
    }
}

/**
  * @brief          ��ʼ��"gimbal_CV"����
  * @param[out]     init: ������init_CV
  * @retval         none
  */
static void CV_init(shoot_control_t *init)
{
		//��ʼ��CV�ṹ��
		init->can_CV = get_can_CV_point();
}

