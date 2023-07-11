# RM2023-infantry
## 使用struct_typedef.h中的宏定义切换环步或张步的条件编译
## 介绍
GKD战队RoboMaster2023赛季，步兵机器人，C板（STM32F407IGHx），FreeRTOS，在官步基础上修改

## 软件和固件库版本
CubeMX 6.4.0  
STM32CubeF4 Firmware Package V1.26.0  

## 现有外设占用
#### 串口
1. USART3：遥控器
2. USART6(板上丝印UART1)：裁判系统
3. USART1(板上丝印UART2)：空闲
#### CAN
1. CAN1：chassis底盘(4个3508)
    - 将增加：超级电容
2. CAN2：视觉，gimbal云台(2个6020), trigger拨弹电机(1个2006)，fric摩擦轮电机(2个3508)  
(摩擦轮电机使用PWM控制的2个3508)

#### PWM
1. PWM0用于控制弹舱盖开关

## 开发记录
- 旧库的开发记录请见原始分支，该开发记录从2023步兵对抗赛结束后开始维护
- 7.11 by Rhine
  - 代码内有冗余部分UI.c/.h和KF.c/.h，分别为UI代码和卡曼滤波代码，后续决定删除还是加入
  - 该分支代码添加了Cmakelists，适配了Mac和Linux环境，修改了代码内部分不兼容错误
    - ~~头文件未区分大小写~~，改为全部区分大小写
    - ~~struct_typedef.h中的定义冲突~~，修改为#include "stdint.h"并define fp32, fp64和 bool_t
    - ~~AHRS闭源库的不适配问题~~，添加了libahrs.a文件进行适配
    - ~~存在一些static修饰的函数被extern到外部~~，删除static
    - ~~dsp库的.lib不适配~~，移植了dsp库的源码，删除了.lib
    - ~~__packed的宏未定义~~，在struct_typedef.h中重新定义
  - 请注意重新生成的cmakelists中一定要开启硬浮点并添加库和源

## 笔记
### gimbal_task 云台控制任务
1. 云台的控制通过结构体**gimbal_control**完成，存放了遥控器/键鼠、陀螺仪、电机的信息
2. **gimbal_set_mode**() -> gimbal_behaviour_mode_set() -> 
   - gimbal_behavour_set(): 云台行为状态机，根据遥控器switch位置确定并通过变量gimbal_behaviour返回当前的云台行为(e.g., GIMBAL_RELATIVE_ANGLE / GIMBAL_ABSOLUTE_ANGLE)
   - 根据变量gimbal_behaviour的值设置每个云台电机的状态，存入结构体中的变量gimbal_motor_mode
3. **gimbal_mode_change_control_transit**() ->
   - 判断结构体gimbal_control中gimbal_motor_mode和last_gimbal_motor_mode是否一致，若不一致，则说明发生了模式切换，即进行数据保存和处理
4. **gimbal_feedback_update**() ->
   - 将CAN总线接收到的各个云台电机的数据进行解码，计算出相对角度、绝对角度、速度等信息，并存入结构体中的相应变量
5. **gimbal_set_control**() ->
   - 新建变量add_yaw_angle和add_pitch_angle，用于存放两个角度的增量值
   - gimbal_behaviour_control_set() ->
     - 根据**云台模式gimbal_behaviour**进入相应的gimbal_xxxxx_control()函数，在函数内使用不同方式根据遥控器/键鼠的操作得到云台pitch和yaw角度的增量，使用指针yaw和pitch返回
   - 根据**电机模式gimbal_motor_mode**决定控制方式，将变量add_yaw_angle和add_pitch_angle中的值存入结构体，用于下面的PID计算
   - 有两种模式参数：云台模式、电机模式
6. **gimbal_control_loop**() ->
   - 根据结构体中gimbal_motor_mode的值进入不同的gimbal_motor_xxxxx_angle_control()函数，根据上一步算出的角度值计算得到current_set/given_current控制值 **Tips.电视联控在这里添加**  
   - *(所有xxx_control_loop函数的逻辑都是相似的)*
7. **shoot_control_loop**() -> (详见程序注释)
   - shoot_set_mode()：状态机函数，用于判断并更新状态
   - shoot_feedback_update()：更新数据
   - 根据状态计算控制量
   - 控制量输出

### chassis_task 底盘控制任务
1. **chassis_task()**
2. **chassis_mode_change_control_transit()**
3. **chassis_feedback_update()**
4. **chassis_set_contorl()** ->
   - 新建三个局部变量，分别存放x轴速度，y轴速度，以及角度设定值
   - chassis_behaviour_control_set() ->
     - 根据**底盘模式chassis_behaviour_mode(单独定义在文件开头的变量)**进入相应的chassis_xxxxx_control()函数，在函数内使用不同方式根据遥控器/键鼠的操作得到vx,vy,angle，使用指针返回数值
     - 根据**电机模式chassis_move_control->chassis_mode**决定控制方式，将三个参数存入结构体，用于下面的PID计算
     - 有两种模式参数：底盘模式、电机模式
5. **chassis_control_loop()** ->
   - PID控制，除了RAW模式之外，其他模式的处理方法都是一样的

### 底盘和云台的协同控制
- relative_angle: 相对角度，来自6020编码器
- absolute_angle: 绝对角度，来自陀螺仪
1. 常用的云台模式:
   1. GIMBAL_RELATIVE_ANGLE: 云台使用**编码器**的相对角度控制旋转量
   2. GIMBAL_ABSOLUTE_ANGLE: 云台使用**陀螺仪**的绝对角度控制旋转量
   3. GIMBAL_CV: 视觉控制模式，坐标差转换为云台角度增量
2. 常用的底盘模式:
   1. CHASSIS_NO_FOLLOW_YAW: 底盘不跟随云台(没有角度闭环)，云台和底盘会根据遥控器摇杆值分别旋转(云台与底盘的角度限制已去掉)，底盘旋转靠双环控制(摇杆值+底盘回正PID)
   2. CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW：底盘通过**编码器**的relative_angle跟随云台(通过旋转底盘使relative_angle保持0)
- 环步使用的控制方式(2022.1.19):
  1. 右switch_up(**小陀螺**): CHASSIS_SPIN(基于CHASSIS_NO_FOLLOW_YAW) + GIMBAL_ABSOLUTE_ANGLE
  2. 右switch_mid(**普通模式**): CHASSIS_NO_FOLLOW_YAW + GIMBAL_ABSOLUTE_ANGLE
  3. 右switch_down(**CV控制**): CHASSIS_NO_FOLLOW_YAW + GIMBAL_CV(基于GIMBAL_ABSOLUTE_ANGLE)
  - *不能使用CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW + GIMBAL_RELATIVE_ANGLE！*

### 云台电机PID
每个轴都有三种PID，会同时使用1和3 或 2和3.
1. gimbal_motor_absolute_angle_pid
2. gimbal_motor_relative_angle_pid
3. gimbal_motor_gyro_pid

### CV控制逻辑
- 使用遥控器逻辑，NX发送摇杆值到32
- 右摇杆置下开启CV

### 操作手控制逻辑
- 平移：遥控器-右摇杆，键鼠-WASD
  - 对于平移，键鼠优先级高于遥控器，使用键鼠控制时会暂时禁用遥控器，一段时间未收到键鼠信号，则启用遥控器
- 视角：遥控器-左摇杆，键鼠-鼠标
  - 对于视角，遥控器和键鼠优先级相同，互不冲突

### 操作手
#### 自定义按键
- SHIFT:  按住开启小陀螺
- CTRL
- Q
- E
- R: 开启/关闭小陀螺
- F: 开启/关闭摩擦轮
- G: 开启/关闭弹舱盖
- Z
- X
- C
- V
- B: 切换点射/连射
#### 学生端按键
- I: 补充42mm弹丸
- K: 使用血包
- F12: 帮助界面

#### 自定义UI

## 参考资料
1. 官方2019步兵开源学习: https://bbs.robomaster.com/forum.php?mod=viewthread&tid=8361
2. C语言之如何输出uint32_t和uint64_t和16进制: https://blog.csdn.net/u011068702/article/details/77938756 
3. 【山东理工大学】自定义UI开源：https://bbs.robomaster.com/forum.php?mod=viewthread&tid=11924
4. Robomatser 画自定义UI界面：https://blog.csdn.net/dbqwcl/article/details/115518917
