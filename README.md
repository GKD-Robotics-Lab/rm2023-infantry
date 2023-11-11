## 版本说明

- 该版本的代码中 `bsp` 与 `components` 基于官方代码 `standard robot` 修改
- `application` 基于 GKD 战队 RMUL2023 步兵代码 `rm2023-infantry-master` 修改
- 该版本旨在精简代码，增加程序的可读性，以及优化部分代码，增加其通用性，另外添加了部分新方法的应用
- @author dokee (dokee.39@gmail.com)

## TODO

- 底盘 `chassis_vector_to_mecanum_wheel_speed` 的改动
- 底盘绝对角度模式的试验
- 看底盘功率控制与键盘控制

- 云台问题
    - 一直连遥控器上电时的突变
    - 摩擦力很大，PID 怎么调或者前馈
    - 两电机同时堵转时限制了功率吗
    - 云台未校准时的摇摆的原因？

- 加速度计等的滤波要看一下

- 测试知陀螺仪加热电阻稳定温度没卵用，换电阻或者控制值直接给最大？

- 使用同一种 PID 结构体 -> 添加前馈和补偿？
- 前馈得改，static 不通用 -> 试试 DSP 库

- 多余任务的删减

- 上电未连接遥控器播放 see you again

## 使用方法

- 该工程建立使用了 VSCode 的插件 EIDE
- 该代码添加了更加详尽和更具有层次感的注释，建议配合插件 Better Comments 使用；该插件的配置见附录

## 改动详情

#### 重构

##### 添加调试宏开关文件 `config.h`

- 蜂鸣器开启宏
- 调试串口开关宏
- 底盘调试宏
- 云台调试宏
- 校准调试宏
- IMU 调试宏

##### pid

- 加入了死区与微分滤波 PID (似乎没大用)
- 加入了引入前馈的接口
- 添加了适用于云台的 `PID_calc_specifyD`，它将使用传入的微分项
- 删除了微分滤波及其使用的 `Dbuf`，如果要尝试微分滤波，可以使用 `PID_calc_specifyD` 传入滤波后的 `D`

##### 底盘 chassis

- `chassis_behaviour.c`
    - 这部分的代码中，其状态机的模式 `chassis_behaviour_mode` 直接对应底盘整体表现的行为模式
    - 该部分代码的主要任务是从遥控器获取机器人的行为模式和遥控器控制数据，并根据不同的控制数据对遥控器控制数据进行对应的处理得到底盘运动控制值
    - 分出该文件的意义是方便添加新模式，尽可能少的修改通用的 `chassis_task.c` 文件
    - 优化了小陀螺模式选择的代码
- `chassis_task.c`
    - 扬了原来的 `chassis_mode`，修改为：
        - 平动策略 `chassis_translation_strategy`：
        ```c
        typedef enum {
            TRANSLATION_RAW,                  // 原始模式 -> 获得值直接赋给运动参数并发送到 CAN 总线上
            TRANSLATION_VECTOR_FOLLOW_BODY,   // 底盘平动方向跟随车身，不跟随云台
            TRANSLATION_VECTOR_FOLLOW_GIMBAL, // 底盘平动方向跟随云台
        } chassis_translation_strategy_e;            
        ```
        - 转动策略 `chassis_rotation_strategy`：
        ```c
        typedef enum {
            ROTATION_DIRECT,   // 直接赋值模式 -> 获得值 angle_set 作为速度直接赋值给 wz_set
            ROTATION_RELATIVE, // 旋转角度相对于云台
            ROTATION_ABSOLUTE, // 旋转角度相对于车身
        } chassis_rotation_strategy_e;
        ```
    - 原来的模式下平动和转动的运动处理在一个模式下，这样分了之后分离了平动和转动的运动控制，这样添加的新模式可以直接分解为平动和转动策略的配合
    - 比如添加的小陀螺模式是 `TRANSLATION_VECTOR_FOLLOW_GIMBAL` 和 `ROTATION_DIRECT` 的配合，不需要再在 `chassis_task.c` 中再添加新模式
    - 添加了方便底盘调试的宏

##### 射击 shoot

- `shoot.c` -> `shoot_task.c`
    - 整个文件几乎全部重构，单列为 freeRTOS 任务，与云台任务分离
    - 对应修改了 `detect` 和 `CAN_receive` 相关内容
    - `shoot_mode_e` 射击状态机：
    ```c
    typedef enum {
        SHOOT_DISABLE = 0,   // 未使能射击时的无力状态
        SHOOT_START,         // 摩擦轮加速阶段
        SHOOT_READY,         // 摩擦轮已准备好，等待发射
        SHOOT_FIRE,          // 发射一个子弹
        SHOOT_CONTINUE_FIRE, // 连击模式
        SHOOT_DONE,          // 子弹发射完后的阶段
        SHOOT_STOP,          // 失能前的摩擦轮减速阶段
    } shoot_mode_e;
    ```
    - 控制逻辑：
        - `shoot_set_mode()`：根据遥控器和键盘设置射击控制模式
            - 摩擦轮控制
                - 上拨判断，一次开启，再次关闭
                - 处于中档，可以使用键盘开启和关闭摩擦轮
            - 射击控制
                - 下拨一次或者鼠标按下一次，进入射击状态
                - 鼠标左键长按进入射击连发状态，松开后退出连发
                - 鼠标右键按下加速摩擦轮
            - 考虑枪口热量限制及云台未启动
        - `shoot_feedback_update()`：射击数据更新
            - 拨弹论速度获取经过了一个低通滤波
            - 拨弹论角度获取数据归算到了输出轴
        - `shoot_switch_mode()`：根据反馈切换射击控制模式
            - `SHOOT_START` -> 加速完成 -> `SHOOT_READY`
            - `SHOOT_FIRE` -> 转到了指定角度 -> `SHOOT_DONE`
            - `SHOOT_DONE` -> 一段延时后记录数据 -> `SHOOT_READY`
            - `SHOOT_STOP` -> 减速完成 -> `SHOOT_DISABLE`
        - `shoot_control_loop()`：根据不同的射击控制模式计算电机控制值
            - 摩擦轮速度控制：直接将斜坡函数输出值作为速度设置值，并使用 PID 计算电流值
            - 拨弹轮速度控制：在拨弹的模式下会进行堵转检测

##### 校准 calibrate

- 得到的校准值加以限制，特别是陀螺仪 -> 0.005
- 如果上电后拨杆和摇杆都在下，就全部校准一遍
- 长按 C 板上的按键 1 秒也可以触发全部校准
- 任务开始后拨杆拨到下也可以通过摇杆选择校准
- 校准时可以用串口发送相应的信息 -> `config.h`
- 校准内容：
    - 温度 -> 每次都校准 -> 快速复位导致 flash  内容丢失
    - 陀螺仪
    - 加速度计、磁传感器 (暂时为空，也没必要校准，但是之后可以玩一下)
    - 云台 -> 扬了
        - 云台本来的校准变成写死
        - 可以用 `config.c` 中的宏开启 debug 查看角度值以方便检查宏参数写的对不对

##### 云台 gimbal

- 云台统一正方向
    - 所有角度的方向最后统一到以云台为参考
        - 云台 Yaw 轴逆时针为正，顺时针为负
        - 云台 Pitch 上转为负，下转为正
        - 这样设置符合一个 x 轴向前、y 轴向左、z 轴向上的右手系，使所有坐标系都是右手系
    - 云台电机
        - 是否反向由宏控制
    - imu
        - 原始位置 (云台水平，两坐标系重合) 时陀螺仪 (云台) 坐标系与欧拉角对应关系
            - `x` <-> `roll`
            - `y` <-> `pitch`
            - `z` <-> `yaw`
        - 在命名中，`x`,`y`,`z` 为相对陀螺仪 (云台) 坐标系，而  `roll`,`pitch`,`yaw` 为相对底盘 (地面) 坐标系
        - 右手系旋转矩阵
            - 平面旋转矩阵：$[x'; y'] = [cos theta, -sin theta; sin theta, cos theta] [x; y]$
            - 绕 x 轴旋转：$[x'; y'; z'] = [1, 0, 0; 0, cos theta, -sin theta; 0, sin theta, cos theta] [x; y; z]$
            - 绕 y 轴旋转：$[x'; y'; z'] = [cos theta, 0, sin theta;0, 1, 0; -sin theta, 0, cos theta] [x; y; z]$ (由于绕 y 轴是 z -> O -> x，所以符号有变化)
            - 绕 z 轴旋转：$[x'; y'; z'] = [cos theta, -sin theta, 0; sin theta, cos theta, 0; 0, 0, 1] [x; y; z]$
        - imu 在 `INS_task.c` 中获取的传感器数据都旋转到与云台坐标系相同，之后再进行姿态解算，旋转过后的坐标系也是右手系
        - 由于我们规定云台坐标系也是右手系，因此不需要在 `gimbal_task.c` 中使用宏规定旋转方向
- `gimbal_task.c`
    - 电机控制模式 `motor_mode`：
        ```c
        typedef enum {
            GIMBAL_MOTOR_RAW = 0, // 电机原始值控制
            GIMBAL_MOTOR_GYRO,    // 电机陀螺仪角度控制
            GIMBAL_MOTOR_GYRO_LIMIT, // 带限位的电机陀螺仪角度控制
            GIMBAL_MOTOR_ENCONDE_LIMIT, // 带限位的电机编码值角度控制
        
            GIMBAL_MOTOR_MODE_LEN,
        } gimbal_motor_mode_e;
        ```
    - 设置值与控制值计算使用了函数数组，简化了代码
        - `gimbal_motor_set_control_func[]()`
        - `gimbal_motor_control_func[]()`
    - `gimbal_motor_gyro_limit_set_control()` 得到的控制值具有『过冲』是因为陀螺仪反馈的延迟
    - 由于陀螺仪初始化大概需要 1 秒，在没有接收到陀螺仪数据之前进入控制循环可能导致云台发疯，因此在 task 开始时检测电机是否全在线时加上了检测陀螺仪是否接收到数据，并且延时一段时间等待 IMU 的解算值稳定下来
- `gimbal_behaviour.c`

#### 小改动

- 删除了部分冗余与陈旧代码
- 删除如 OLED、KF 等废弃代码
- usart
    - usart1 波特率改为 921600，开启中断，删除了原有的启动代码等
    - 移植了适用于嵌入式系统的 printf
- `main.c`
    - 添加 `bsp_buzzer_init()`
- `bsp_led.c`, `led_flow_task.c`
    - 加入初始化代码
- `bsp_buzzer.c`
    - 添加初始化代码，并加入宏控制 buzzer 的开关 
- `user_lib.c`
    - 如 `ABS()` 等通用的代码改为从该文件中调用
- 删除有关 laser 的代码
- 移植了适用于嵌入式系统的 printf，来源：[GitHub - mpaland/printf](https://github.com/mpaland/printf)

## 附录

#### Better Comments 插件配置

``` json
"better-comments.tags": [
        {
            "tag": "!",
            "color": "#D3FFA9",
            "strikethrough": false,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
        {
            "tag": "?",
            "color": "#FAAFBE",
            "strikethrough": false,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
        {
            "tag": "//",
            "color": "#474747",
            "strikethrough": true,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
        {
            "tag": "todo",
            "color": "#FF8C00",
            "strikethrough": false,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
        {
            "tag": "*",
            "color": "#98C379",
            "strikethrough": false,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
        {
            "tag": ">",
            "color": "#F2D557",
            "strikethrough": false,
            "underline": false,
            "backgroundColor": "transparent",
            "bold": false,
            "italic": false
        },
    ],
```

#### Better Comments 插件测试用例

``` c
/*
     test
*    test
!    test
?    test
>    test
todo test
//   test
*/
```
