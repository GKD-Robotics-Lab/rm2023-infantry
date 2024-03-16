#include "referee.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "UI.h"
#include "custom_ui_task.h"

Crosshair_Data_Type Crosshair_Data;

void draw_crosshair();
void ui_parameter_init();


void custom_ui_task(void const * argument)
{
    ui_parameter_init();

    for (;;)
    {
        draw_crosshair();
        // Graph_Data test_line;

        // Line_Draw(&test_line, "TS", UI_Graph_ADD, 0, UI_Color_Yellow, 10, 300, 300
        //     ,800, 800);
        
        // UI_ReFresh(1, test_line);


        osDelay(100); //ˢ����=10Hz
    }
}

void draw_crosshair()
{
    /*����ֹ׼��*/
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
    //UI_ReFresh(2, still_cross_line[0], still_cross_line[1]);
    //���
}

void update_cross_paramater()
{

}

void ui_parameter_init()
{
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

    Crosshair_Data.line_width = 3;     //�߿��
    Crosshair_Data.cross_colar = UI_Color_Orange;   //׼����ɫ
    Crosshair_Data.ruler_colar = UI_Color_Yellow;   //�����ɫ

    Crosshair_Data.dist_indicate_color = UI_Color_Purplish_red; //������ɫ
    Crosshair_Data.dist_indicate_length = 150;  //���߳���
    Crosshair_Data.dist_indicate_width = 20;    //���߿��

    Crosshair_Data.distance = 0;    //Ĭ�Ͼ���=0


    /*���&������ʾ*/
    Crosshair_Data.dist_start_point[0] = 1260;      //�����ʼX
    Crosshair_Data.dist_start_point[0] = 640;       //�����ʼY
    Crosshair_Data.dist_display_length = 200;       //����������
    Crosshair_Data.dist_display_width = 20;        //���������
    
    Crosshair_Data.dist_start_point[0] = 1260;      //������ʼX
    Crosshair_Data.dist_start_point[0] = 440;       //������ʼY
    Crosshair_Data.dist_display_length = 200;       //����������
    Crosshair_Data.dist_display_width = 20;         //���������


}
