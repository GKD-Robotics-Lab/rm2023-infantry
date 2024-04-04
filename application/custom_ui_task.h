#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "UI.h"

/*准星中心位置*/
#define CROSS_CENTER_X 960
#define CROSS_CENTER_Y 540

/*弹道标定*/
#define CROSS_1M 100
#define CROSS_2M 200
#define CROSS_3M 300
#define CROSS_4M 400
#define CROSS_5M 500

/*机器人角色*/
#define Robot_ID UI_Data_RobotID_RHero
#define Cilent_ID UI_Data_CilentID_RHero

/*UI模式*/
#define UI_INFANTRY 1
#define UI_HERO 2
#define UI_MODE UI_HERO

#define RE_INIT_CYCLE 30

/*摩擦轮状态*/
#define FRIC_OFF 0
#define FRIC_ON  1
#define FRIC_ACC 2

/*UI显示参数的结构体*/
typedef struct
{
    float distance;     //目标距离
    float shoot_speed;  //射速
    float Super_cap_percent;    //超电百分比
    int spin_state;     //小陀螺状态
    int fric_state;     //摩擦轮状态
    int auto_aim_state; //自瞄状态
} UI_DisplayData_Type;

/*描述准星的结构体*/
typedef struct
{
    /*准星本体*/
    u32 center[2]; //准星中心
    u32 ballistic_ruler[5]; //弹道标尺，分别为1m~5m落点相对准星下降的距离
    u32 ruler_length[5]; //标尺的长度
    u32 line_width; //线宽度
    u32 cross_width;
    u32 cross_high; //十字高度
    u32 cross_high_offset; //十字高度偏移
    u32 cross_colar;    //准星颜色
    u32 ruler_colar;    //标尺颜色
    u32 dist_indicate_color;    //测距指示颜色
    u32 dist_indicate_length;   //测距指示长度
    u32 dist_indicate_width;   //测距指示宽度
    u32 distance;   //测得距离

    /*测距&弹速显示*/
    u32 dist_start_point[2];    //测距文字起始位置
    u32 dist_text_size;         //测速文字字号
    u32 dist_display_length;    //测速条的长度
    u32 dist_display_width;     //测速条宽度

    u32 speed_start_point[2];    //射速文字起始位置
    u32 speed_display_length;    //射速条的长度
    u32 speed_display_width;     //射速条宽度
    u32 speed_text_size;

    u32 shoot_text_color;       //文字颜色
    u32 shoot_bar_color;        //百分条颜色

    u32 shoot_speed_percent;
    u32 shoot_dist_percent;
} Crosshair_Data_Type;

/*描述右下角状态指示的结构体*/
typedef struct
{
    /* 超电部分 */
    u32 cap_text_pos[2];    //超电字体位置，左下角
    u32 cap_display_length; //容量条长度
    u32 cap_display_with;   //容量条宽度
    u32 cap_text_size;      //超电字号
    u32 cap_text_color;     //字体颜色
    u32 cap_bar_color;      //百分条颜色
    int cap_percent;        //超电百分比

    /* 状态部分 */
    u32 spin_state_pos[2];
    u32 fric_state_pos[2];
    int spin_state;
    int fric_state;
} State_Indicate_Type;



extern void custom_ui_task(void const * argument);
extern void custom_UI_init();
extern UI_DisplayData_Type UI_Data;