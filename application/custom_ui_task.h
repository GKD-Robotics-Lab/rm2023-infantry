#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "UI.h"

/*UI��ʾ�����Ľṹ��*/
typedef struct
{
    float distance;     //Ŀ�����
    float shoot_speed;  //����
    float Super_cap_percent;    //����ٷֱ�
    int spin_state;     //С����״̬
    int fric_state;     //Ħ����״̬
} UI_DisplayData_Type;

/*����׼�ǵĽṹ��*/
typedef struct
{
    /*׼�Ǳ���*/
    u32 center[2]; //׼������
    u32 ballistic_ruler[5]; //������ߣ��ֱ�Ϊ1m~5m������׼���½��ľ���
    u32 ruler_length[5]; //��ߵĳ���
    u32 line_width; //�߿��
    u32 cross_width;
    u32 cross_high; //ʮ�ָ߶�
    u32 cross_high_offset; //ʮ�ָ߶�ƫ��
    u32 cross_colar;    //׼����ɫ
    u32 ruler_colar;    //�����ɫ
    u32 dist_indicate_color;    //���ָʾ��ɫ
    u32 dist_indicate_length;   //���ָʾ����
    u32 dist_indicate_width;   //���ָʾ���
    u32 distance;   //��þ���

    /*���&������ʾ*/
    u32 dist_start_point[2];    //���������ʼλ��
    u32 dist_text_size;         //���������ֺ�
    u32 dist_display_length;    //�������ĳ���
    u32 dist_display_width;     //���������

    u32 speed_start_point[2];    //����������ʼλ��
    u32 speed_display_length;    //�������ĳ���
    u32 speed_display_width;     //���������
    u32 speed_text_size;

    u32 shoot_text_color;       //������ɫ
    u32 shoot_bar_color;        //�ٷ�����ɫ

    u32 shoot_speed_percent;
    u32 shoot_dist_percent;
} Crosshair_Data_Type;

/*�������½�״ָ̬ʾ�Ľṹ��*/
typedef struct
{
    /* ���粿�� */
    u32 cap_text_pos[2];    //��������λ�ã����½�
    u32 cap_display_length; //����������
    u32 cap_display_with;   //���������
    u32 cap_text_size;      //�����ֺ�
    u32 cap_text_color;     //������ɫ
    u32 cap_bar_color;      //�ٷ�����ɫ
    int cap_percent;        //����ٷֱ�

    /* ״̬���� */
    u32 spin_state_pos[2];
    u32 fric_state_pos[2];
    int spin_state;
    int fric_state;
} State_Indicate_Type;



extern void custom_ui_task(void const * argument);