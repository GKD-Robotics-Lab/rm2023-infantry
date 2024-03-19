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

void draw_crosshair_hero();
void draw_crosshair_infantry(); 
void update_dynamic_paramater();
void UI_clear();
void custom_UI_init();
void int_to_str(char *to_str, int number);
void cap_text_format(char *to_str, int cap_percent);
void spin_state_str(char *to_str, int spin_state);
void fric_state_str(char *to_str, int fric_state);
void ui_parameter_init();
void cap_text_format(char *to_str, int cap_percent);
void sync_parameter();
void state_str(char *to_str, int cap_percent, int spin_state, int fric_state);
void UI_init_draw();


String_Data state_text_data;
Graph_Data shoot_distance_bar, cap_percentage;
Graph_Data still_cross_line[7];
char cap_text[30];
int count = 0;     //计数器


void custom_ui_task(void const * argument)
{
    custom_UI_init();
    sync_parameter();
    UI_init_draw();

    while(1)
    {
        sync_parameter();
        update_dynamic_paramater();

        //间隔一定时间重新初始化ui
        if(count >= RE_INIT_CYCLE)
        {
            osDelay(100);
            if(UI_MODE == UI_HERO) draw_crosshair_hero();
            else if(UI_MODE == UI_INFANTRY) draw_crosshair_infantry();
            osDelay(100);
            UI_init_draw();
            count = 0;
        } 
        count ++;

        //测试刷新用
        UI_Data.Super_cap_percent+=2;
        if(UI_Data.Super_cap_percent>=100) UI_Data.Super_cap_percent=0;
        if(UI_Data.Super_cap_percent>=50) UI_Data.spin_state = 1;
        else UI_Data.spin_state = 0;
        if(UI_Data.Super_cap_percent>=70) UI_Data.fric_state = 1;
        else UI_Data.fric_state = 0;

        osDelay(100); //刷新率=10Hz
    }
}

/*画英雄的静止准星*/
void draw_crosshair_hero()
{
    char line_id[7][3] = {"L1", "L2", "L3", "L4", "L5", "L6", "L7"};
    /*准星*/
    //横着的主准星
    Line_Draw(&still_cross_line[0], "L1", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0]-(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1],
        Crosshair_Data.center[0]+(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1]);
    //竖着的主准星
    Line_Draw(&still_cross_line[1], "L2", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]+(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]-(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset);
    //5个标尺
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
    
    osDelay(100); //确保间隔
    //标尺
}

/*画英雄的静步兵准星*/
void draw_crosshair_infantry()
{
    /*准星*/
    //横着的主准星
    Line_Draw(&still_cross_line[0], "L1", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0]-(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1],
        Crosshair_Data.center[0]+(Crosshair_Data.cross_width/2),
        Crosshair_Data.center[1]);
    //竖着的主准星
    Line_Draw(&still_cross_line[1], "L2", UI_Graph_ADD, 0, 
        Crosshair_Data.cross_colar,
        Crosshair_Data.line_width,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]+(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset,
        Crosshair_Data.center[0],
        Crosshair_Data.center[1]-(Crosshair_Data.cross_high/2)+Crosshair_Data.cross_high_offset);
    UI_ReFresh(2, still_cross_line[0], still_cross_line[1]);
    
    osDelay(100); //确保间隔
    //标尺
}

void UI_init_draw()
{
    //测距部分的刷新
    Line_Draw(&shoot_distance_bar, "dst", UI_Graph_ADD, 1, Crosshair_Data.shoot_bar_color,
        Crosshair_Data.dist_display_width, 
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width,
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0] + ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width);

    /*刷新超电部分*/
    Line_Draw(&cap_percentage, "cap", UI_Graph_ADD, 1, State_Data.cap_bar_color, State_Data.cap_display_with,
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1] - State_Data.cap_text_size*4.8,
                State_Data.cap_text_pos[0] + ((State_Data.cap_display_length*State_Data.cap_percent)/100),
                State_Data.cap_text_pos[1] - State_Data.cap_text_size*4.8);
    UI_ReFresh(2, shoot_distance_bar, cap_percentage);
    osDelay(100);
    state_str(cap_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
    
    String_Draw(&state_text_data, "sta", UI_Graph_ADD, 1, State_Data.cap_text_color,
                State_Data.cap_text_size, 21, 2, 
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1], cap_text);
    String_ReFresh(state_text_data);
    osDelay(100);
}

/*刷新动态参数*/
void update_dynamic_paramater()
{

    //测距部分的刷新
    Line_Draw(&shoot_distance_bar, "dst", UI_Graph_Change, 1, Crosshair_Data.shoot_bar_color,
        Crosshair_Data.dist_display_width, 
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width,
        Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0] + ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
        Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width);

    //超电的刷新
    Line_Draw(&cap_percentage, "cap", UI_Graph_Change, 1, State_Data.cap_bar_color, State_Data.cap_display_with,
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1] - State_Data.cap_text_size*4.8,
                State_Data.cap_text_pos[0] + ((State_Data.cap_display_length*State_Data.cap_percent)/100),
                State_Data.cap_text_pos[1] - State_Data.cap_text_size*4.8);
    //状态的刷新
    state_str(cap_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
    String_Draw(&state_text_data, "sta", UI_Graph_Change, 1, State_Data.cap_text_color,
                State_Data.cap_text_size, 21, 2, 
                State_Data.cap_text_pos[0],
                State_Data.cap_text_pos[1], cap_text);

    //应用刷新(英雄显示测距和准星，步兵无准星)
    if(UI_MODE == UI_HERO)  UI_ReFresh(2, shoot_distance_bar, cap_percentage);
    else if(UI_MODE == UI_INFANTRY) UI_ReFresh(1, cap_percentage);
    osDelay(100);
    String_ReFresh(state_text_data);
 }

/*刷新动态参数*/
// void update_dynamic_paramater()
// {
//     Graph_Data shoot_distance_bar, cap_percentage;
//     String_Data state_text_data;
//     char state_text[30];

//     //测距部分的刷新
//     Line_Draw(&shoot_distance_bar, "dst", UI_Graph_Change, 1, Crosshair_Data.shoot_bar_color,
//         Crosshair_Data.dist_display_width, 
//         Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0],
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width,
//         Crosshair_Data.center[0] + Crosshair_Data.dist_start_point[0] + ((Crosshair_Data.dist_display_length*Crosshair_Data.shoot_dist_percent)/100),
//         Crosshair_Data.center[1] + Crosshair_Data.dist_start_point[1] - Crosshair_Data.dist_display_width);

//     /*刷新超电部分*/
//     Line_Draw(&cap_percentage, "cap", UI_Graph_Change, 1, State_Data.cap_bar_color, State_Data.cap_display_with,
//                 State_Data.cap_text_pos[0],
//                 State_Data.cap_text_pos[1] - State_Data.cap_text_size,
//                 State_Data.cap_text_pos[0] + ((State_Data.cap_display_length*State_Data.cap_percent)/100),
//                 State_Data.cap_text_pos[1] - State_Data.cap_text_size);
//     UI_ReFresh(2, shoot_distance_bar, cap_percentage);
//     state_str(&state_text, State_Data.cap_percent, State_Data.spin_state, State_Data.fric_state);
//     // osDelay(110);
//     // String_ReFresh(state_text_data);
//  }

void UI_clear()
{
    UI_Delete(UI_Data_Del_ALL, 0);
    osDelay(100);
}

void custom_UI_init()
{
    UI_clear();
    ui_parameter_init();
    if(UI_MODE == UI_HERO) draw_crosshair_hero();
    else if(UI_MODE == UI_INFANTRY) draw_crosshair_infantry();
}

void int_to_str(char *to_str, int number)
{
    to_str[0] = ((number/100) % 10) + 48;
    to_str[1] = ((number/10) % 10) + 48;
    to_str[2] = (number % 10) + 48;
    to_str[3] = '\0';
}

//生成超电百分比文字
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

//生成自旋字符串
void spin_state_str(char *to_str, int spin_state)
{
    to_str[0] = 'S';
    to_str[1] = 'P';
    to_str[2] = 'I';
    to_str[3] = 'N';
    to_str[4] = ' ';
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

//生成摩擦轮字符串
void fric_state_str(char *to_str, int fric_state)
{
    to_str[0] = 'F';
    to_str[1] = 'R';
    to_str[2] = 'I';
    to_str[3] = 'C';
    to_str[4] = ' ';
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
    
    cap_text_format(cap_str, cap_percent);
    spin_state_str(spin_str, spin_state);
    fric_state_str(fric_str, fric_state);
    for(int i=0; i<8; i++)  to_str[i] = fric_str[i];
    to_str[8] = '\n';
    for(int i=9; i<17; i++) to_str[i] = spin_str[i-9];
    to_str[17] = '\n';

    //字符串长度限制，改成电容只显示三位数字
    to_str[18] = ((cap_percent/100) % 10) + 48;
    to_str[19] = ((cap_percent/10) % 10) + 48;
    to_str[20] = (cap_percent % 10) + 48;
}

//同步UI_DisplayData_Type的状态到UI控制结构体
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

    /*准星参数初始化*/
    Crosshair_Data.center[0] = CROSS_CENTER_X;     //中心X
    Crosshair_Data.center[1] = CROSS_CENTER_Y;     //中心Y
    Crosshair_Data.cross_width = 500;   //宽
    Crosshair_Data.cross_high = 700;   //高
    Crosshair_Data.cross_high_offset = -100; //高偏移

    Crosshair_Data.ballistic_ruler[0] = CROSS_1M;     //1m标尺
    Crosshair_Data.ballistic_ruler[1] = CROSS_2M;     //2m标尺
    Crosshair_Data.ballistic_ruler[2] = CROSS_3M;     //3m标尺
    Crosshair_Data.ballistic_ruler[3] = CROSS_4M;     //4m标尺
    Crosshair_Data.ballistic_ruler[4] = CROSS_5M;     //5m标尺

    Crosshair_Data.ruler_length[0] = 400;   //1m标尺长度
    Crosshair_Data.ruler_length[1] = 300;   //2m标尺长度
    Crosshair_Data.ruler_length[2] = 200;   //3m标尺长度
    Crosshair_Data.ruler_length[3] = 100;   //4m标尺长度
    Crosshair_Data.ruler_length[4] = 50;   //5m标尺长度

    Crosshair_Data.line_width = 2;     //线宽度
    Crosshair_Data.cross_colar = UI_Color_Orange;   //准星颜色
    Crosshair_Data.ruler_colar = UI_Color_Yellow;   //标尺颜色

    Crosshair_Data.dist_indicate_color = UI_Color_Purplish_red; //测距尺颜色
    Crosshair_Data.dist_indicate_length = 150;  //测距尺长度
    Crosshair_Data.dist_indicate_width = 30;    //测距尺宽度

    Crosshair_Data.distance = 0;    //默认距离=0


    /*测距&射速显示*/
    Crosshair_Data.dist_start_point[0] = 140;      //测距起始X(相对中心)
    Crosshair_Data.dist_start_point[1] = 20;       //测距起始Y(相对中心)
    Crosshair_Data.dist_display_length = 100;       //测速条长度
    Crosshair_Data.dist_display_width = 8;        //测速条宽度
    Crosshair_Data.dist_text_size = 15;             //测速文字字号
    Crosshair_Data.shoot_dist_percent = 10;

    Crosshair_Data.speed_start_point[0] = 140;      //测距起始X(相对中心)
    Crosshair_Data.speed_start_point[1] = -30;       //测距起始Y(相对中心)
    Crosshair_Data.speed_display_length = 100;       //射速条长度
    Crosshair_Data.speed_display_width = 8;         //射速条宽度
    Crosshair_Data.speed_text_size = 15;            //射速条字体大小
    Crosshair_Data.shoot_speed_percent = 10;

    Crosshair_Data.shoot_text_color = UI_Color_Orange;  //字体颜色  
    Crosshair_Data.shoot_bar_color = UI_Color_Cyan;     //白分条颜色

    State_Data.cap_text_pos[0] = 1600;      //超电字体X
    State_Data.cap_text_pos[1] = 800;       //超电字体Y
    State_Data.cap_display_with = 30;       //超电条宽度
    State_Data.cap_display_length = 250;    //超电条长度
    State_Data.cap_text_size = 30;          //字体大小
    State_Data.cap_text_color = UI_Color_Orange;    //文字颜色
    State_Data.cap_bar_color = UI_Color_Orange;     //百分条颜色

    State_Data.cap_percent = 100;

    State_Data.fric_state = 0;  //摩擦轮开关状态
    State_Data.spin_state = 0;  //小陀螺开关状态

    /*动态参数*/
    UI_Data.distance = 0.0;           //距离
    UI_Data.shoot_speed = 0.0;        //弹速
    UI_Data.Super_cap_percent = 35.0;  //超电百分比
    UI_Data.spin_state = 0;         //自旋状态
    UI_Data.fric_state = 0;         //摩擦轮状态
}
