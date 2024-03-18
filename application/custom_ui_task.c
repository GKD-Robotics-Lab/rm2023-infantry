#include "referee.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "UI.h"
#include "custom_ui_task.h"
#include "stdio.h"
#include "string.h"

Crosshair_Data_Type Crosshair_Data;
UI_DisplayData_Type UI_Data;
State_Indicate_Type State_Data;

void draw_crosshair();
void ui_parameter_init();
void update_dynamic_paramater();
void UI_clear();
void cap_text_format(char *to_str, int cap_percent);

void custom_ui_task(void const * argument)
{
    custom_UI_init();
    for (;;)
    {
        sync_parameter();
        // Crosshair_Data.shoot_dist_percent = (u32)(UI_Data.distance*(100/8));
        // State_Data.cap_percent = (u32)(UI_Data.distance*(100/8));
        update_dynamic_paramater();
        
        UI_Data.distance += 0.1; 
        if(UI_Data.distance >= 8.0) UI_Data.distance=0.0;

        osDelay(100); //ˢ����=10Hz
    }
}

/*����ֹ׼��*/
void draw_crosshair()
{
    Graph_Data still_cross_line[7];
    char line_id[7][3] = {"L1", "L2", "L3", "L4", "L5", "L6", "L7"};
    /*׼��*/
    //���ŵ���׼��
    Line_Draw(&still_cross_line[0], "L1", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0]-(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1],
        Crosshair_Data.center[0]+(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1]);
    //���ŵ���׼��
    Line_Draw(&still_cross_line[1], "L2", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]+(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]-(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset);
    //5�����
    for(int i=0; i<5; i++)
    {
        Line_Draw(&still_cross_line[i+2], line_id[i+2], UI_Graph_ADD, 0, 
            Crosshair_Data.ruler_colar,
            Crosshair_Data.line_width,
            Crosshair_Data.center[0]-(Crosshair_Data.ruler_length[i]/2),
            Crosshair_Data.center[1]-(Crosshair_Data.ballistic_ruler[i]/2),
            Crosshair_Data.center[0]+(Crosshair_Data.ruler_length[i]/2),
            Crosshair_Data.center[1]-(Crosshair_Data.ballistic_ruler[i]/2));
    }
    UI_ReFresh(7, still_cross_line[0], still_cross_line[1], still_cross_line[2],
        still_cross_line[3], still_cross_line[4], still_cross_line[5],
        still_cross_line[6]);
    
    osDelay(100); //ȷ�����
    //���
}

/*���������*/
// void draw_shoot_paramater()
// {
//     String_Data dist_text;
//     Graph_Data dist_bar;
//     //���ƾ���  
//     String_Draw(&dist_text, "C1", UI_Graph_ADD, 1, Crosshair_Data.shoot_text_color,
//         Crosshair_Data.dist_text_size, 4, 30, 
//         Crosshair_Data.dist_start_point[0] + Crosshair_Data.center[0], 
//         Crosshair_Data.dist_start_point[1] + Crosshair_Data.center[1], "1.4m");       
//     String_ReFresh(dist_text);

//     Line_Draw(&dist_bar, "DS", UI_Graph_ADD, 1, Crosshair_Data.shoot_bar_color,
//         Crosshair_Data.dist_display_width, 
//         Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width,
//         Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0] + ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width);

//     UI_ReFresh(1, dist_bar);
//     osDelay(100);
// }

/*ˢ�¶�̬����*/
void update_dynamic_paramater()
{
    /*׼��ˢ�²���*/
    Graph_Data shoot_distance_bar, cap_percentage;
    String_Data shoot_distance_text, cap_percent_text;
    char dist_text[5], cap_text[10];

    //ɾ������Ϣ
    UI_Delete(UI_Data_Del_Layer, 1); 

    //��ಿ�ֵ�ˢ��
    int_to_str(&dist_text, (int)(UI_Data.distance*100));
    Line_Draw(&shoot_distance_bar, "DS", UI_Graph_ADD, 1, Crosshair_Data.shoot_bar_color,
        Crosshair_Data.dist_display_width, 
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width,
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0] + ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width);
    // String_Draw(&shoot_distance_text, "S1", UI_Graph_ADD, 1, Crosshair_Data.shoot_text_color,
    // Crosshair_Data.dist_text_size, 3, 30, 
    // Crosshair_Data.dist_start_point[0] + Crosshair_Data.center[0], 
    // Crosshair_Data.dist_start_point[1] + Crosshair_Data.center[1], &dist_text);

    /*ˢ�³��粿��*/
    Line_Draw(&cap_percentage, "CP", UI_Graph_ADD, 1, State_Data.cap_bar_color, State_Data.cap_display_with,
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1] - State_Data.cap_text_size,
                State_Data.cap_text_pos[0] + ((State_Data.cap_display_length*State_Data.cap_percent)/100),
                State_Data.cap_text_pos[1] - State_Data.cap_text_size);
    
    state_str(&cap_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
    String_Draw(&cap_percent_text, "CI", UI_Graph_ADD, 1, State_Data.cap_text_color,
                State_Data.cap_text_size, 21, 30, 
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1], &cap_text);

    // //Ӧ��ˢ��
    UI_ReFresh(2, shoot_distance_bar, cap_percentage);
    // String_ReFresh(shoot_distance_text);
    String_ReFresh(cap_percent_text);
 }

void UI_clear()
{
    UI_Delete(UI_Data_Del_ALL, 0);
    osDelay(100);
}

void custom_UI_init()
{
    UI_clear();
    ui_parameter_init();
    draw_crosshair();
    //draw_shoot_paramater();
}

void int_to_str(char *to_str, int number)
{
    to_str[0] = ((number/100) % 10) + 48;
    to_str[1] = ((number/10) % 10) + 48;
    to_str[2] = (number % 10) + 48;
    to_str[3] = '\0';
}

//���ɳ���ٷֱ�����
void cap_text_format(char *to_str, int cap_percent)
{
    to_str[0] = 'C';
    to_str[1] = 'A';
    to_str[2] = 'P';
    to_str[3] = ':';
    to_str[4] = ' ';
    to_str[5] = ((cap_percent/100) % 10) + 48;
    to_str[6] = ((cap_percent/10) % 10) + 48;
    to_str[7] = (cap_percent % 10) + 48;
    to_str[8] = '%';
    to_str[9] = '\0';
}

//���������ַ���
void spin_state_str(char *to_str, int spin_state)
{
    to_str[0] = 'S';
    to_str[1] = 'P';
    to_str[2] = 'I';
    to_str[3] = 'N';
    to_str[4] = '   ';
    if(spin_state == 0)
    {
        to_str[5] = 'O';
        to_str[6] = 'F';
        to_str[7] = 'F';
        to_str[8] = '\0';
    }
    else if(spin_state == 1)
    {
        to_str[5] = 'O';
        to_str[6] = 'N';
        to_str[7] = ' ';
        to_str[8] = '\0';
    }
}

//����Ħ�����ַ���
void fric_state_str(char *to_str, int fric_state)
{
    to_str[0] = 'F';
    to_str[1] = 'R';
    to_str[2] = 'I';
    to_str[3] = 'C';
    to_str[4] = '   ';
    if(fric_state == 0)
    {
        to_str[5] = 'O';
        to_str[6] = 'F';
        to_str[7] = 'F';
        to_str[8] = '\0';
    }
    else if(fric_state == 1)
    {
        to_str[5] = 'O';
        to_str[6] = 'N';
        to_str[7] = ' ';
        to_str[8] = '\0';
    }
}

void state_str(char *to_str, int cap_percent, int spin_state, int fric_state)
{
    char cap_str[10];
    char spin_str[10];
    char fric_str[10];
    int index = 0;
    
    cap_text_format(&cap_str, cap_percent);
    spin_state_str(&spin_str, spin_state);
    fric_state_str(&fric_str, fric_state);
    for(int i=0; i<8; i++)  to_str[i] = fric_str[i];
    to_str[8] = '\n';
    for(int i=9; i<17; i++) to_str[i] = spin_str[i-9];
    to_str[17] = '\n';

    //�ַ������ƣ��ĳɵ���ֻ��ʾ��λ����
    to_str[18] = ((cap_percent/100) % 10) + 48;
    to_str[19] = ((cap_percent/10) % 10) + 48;
    to_str[20] = (cap_percent % 10) + 48;
    //for(int i=18; i<29; i++) to_str[i] = cap_str[i-18];
    //to_str[27] = '\0';

    // for(int i=0; i<9; i++)  to_str[i] = fric_str[i];

    
}

//ͬ��UI_DisplayData_Type��״̬��UI���ƽṹ��
void sync_parameter()
{
    Crosshair_Data.shoot_dist_percent = (u32)(UI_Data.distance*(100/8));
    State_Data.cap_percent = (u32)(UI_Data.Super_cap_percent);
    State_Data.fric_state = UI_Data.fric_state;
    State_Data.spin_state = UI_Data.spin_state;
}

void ui_parameter_init()
{
    UI_Data.distance = 0.0;

    /*׼�ǲ�����ʼ��*/
    Crosshair_Data.center[0] = 960;     //����X
    Crosshair_Data.center[1] = 540;     //����Y
    Crosshair_Data.cross_width = 500;   //��
    Crosshair_Data.cross_high = 700;   //��
    Crosshair_Data.cross_high_offset = -100; //��ƫ��

    Crosshair_Data.ballistic_ruler[0] = 100;     //1m���
    Crosshair_Data.ballistic_ruler[1] = 200;     //2m���
    Crosshair_Data.ballistic_ruler[2] = 300;     //3m���
    Crosshair_Data.ballistic_ruler[3] = 400;     //4m���
    Crosshair_Data.ballistic_ruler[4] = 500;     //5m���

    Crosshair_Data.ruler_length[0] = 400;   //1m��߳���
    Crosshair_Data.ruler_length[1] = 300;   //2m��߳���
    Crosshair_Data.ruler_length[2] = 200;   //3m��߳���
    Crosshair_Data.ruler_length[3] = 100;   //4m��߳���
    Crosshair_Data.ruler_length[4] = 50;   //5m��߳���

    Crosshair_Data.line_width = 2;     //�߿��
    Crosshair_Data.cross_colar = UI_Color_Orange;   //׼����ɫ
    Crosshair_Data.ruler_colar = UI_Color_Yellow;   //�����ɫ

    Crosshair_Data.dist_indicate_color = UI_Color_Purplish_red; //������ɫ
    Crosshair_Data.dist_indicate_length = 150;  //���߳���
    Crosshair_Data.dist_indicate_width = 30;    //���߿��

    Crosshair_Data.distance = 0;    //Ĭ�Ͼ���=0


    /*���&������ʾ*/
    Crosshair_Data.dist_start_point[0] = 140;      //�����ʼX(�������)
    Crosshair_Data.dist_start_point[1] = 20;       //�����ʼY(�������)
    Crosshair_Data.dist_display_length = 100;       //����������
    Crosshair_Data.dist_display_width = 8;        //���������
    Crosshair_Data.dist_text_size = 15;             //���������ֺ�
    Crosshair_Data.shoot_dist_percent = 10;

    Crosshair_Data.speed_start_point[0] = 140;      //�����ʼX(�������)
    Crosshair_Data.speed_start_point[1] = -30;       //�����ʼY(�������)
    Crosshair_Data.speed_display_length = 100;       //����������
    Crosshair_Data.speed_display_width = 8;         //���������
    Crosshair_Data.speed_text_size = 15;            //�����������С
    Crosshair_Data.shoot_speed_percent = 10;

    Crosshair_Data.shoot_text_color = UI_Color_Orange;  //������ɫ  
    Crosshair_Data.shoot_bar_color = UI_Color_Cyan;     //�׷�����ɫ

    State_Data.cap_text_pos[0] = 1500;      //��������X
    State_Data.cap_text_pos[1] = 100;       //��������Y
    State_Data.cap_display_with = 30;       //���������
    State_Data.cap_display_length = 250;    //����������
    State_Data.cap_text_size = 30;          //�����С
    State_Data.cap_text_color = UI_Color_Orange;    //������ɫ
    State_Data.cap_bar_color = UI_Color_Orange;     //�ٷ�����ɫ

    State_Data.cap_percent = 100;

    State_Data.fric_state = 0;  //Ħ���ֿ���״̬
    State_Data.spin_state = 0;  //С���ݿ���״̬

    /*��̬����*/
    UI_Data.distance = 0.0;           //����
    UI_Data.shoot_speed = 0.0;        //����
    UI_Data.Super_cap_percent = 35.0;  //����ٷֱ�
    UI_Data.spin_state = 0;         //����״̬
    UI_Data.fric_state = 0;         //Ħ����״̬
}
