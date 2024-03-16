#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "UI.h"


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
    u32 dist_display_length;    //测速条的长度
    u32 dist_display_width;     //测速条宽度

    u32 speed_start_point[2];    //射速文字起始位置
    u32 speed_display_length;    //射速条的长度
    u32 speed_display_width;     //射速条宽度


} Crosshair_Data_Type;

/*描述右下角状态指示的结构体*/
typedef struct
{
    /* 超电部分 */
    u32 cap_text_pos[2];    //超电字体位置，左下角
    u32 cap_display_length; //容量条长度
    u32 cap_display_with;   //容量条宽度
    /* 状态部分 */
    u32 spin_state_pos[2];
} State_Indicate_Type;



extern void custom_ui_task(void const * argument);