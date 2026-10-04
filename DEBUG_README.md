# ROBOT_FIRST 调试指南

本指南用于 STM32F407 机器人底盘的首次上电、通信联调和故障定位。

> 警告：首次调试必须让履带悬空、关节无负载，并准备物理急停或能快速断开电机电源。不要在地面直接使能电机。

## 1. 当前软件结构

控制链路如下：

```text
遥控器 → USART3 + DMA → remote.c
                         ↓
TIM6（2 ms） → Robot_Task() → DJI 电机 / DM4310 / 舵机
CAN1 RX0 ───────────────────→ M3508、M2006、GM6020 反馈
CAN2 RX0 ───────────────────→ DM4310 反馈
```

启动时，`main()` 在完成 GPIO、DMA、CAN、TIM、USART 初始化后：

1. 调用 `Robot_Init()`；
2. 启动 CAN 接收、中断通知、遥控 DMA 和 3 路舵机 PWM；
3. 启动 TIM6 更新中断；
4. 每 2 ms 在 `HAL_TIM_PeriodElapsedCallback()` 中调用 `Robot_Task()`。

## 2. 构建与烧录

### CMake 构建

在工程根目录执行：

```powershell
cmake -S . -B build -G Ninja
cmake --build build
```

生成文件为 `build/ROBOT_FIRST.elf`。工程已将 `User/Src` 下的控制模块和 `User/Inc` 头文件目录纳入构建。

### 烧录前检查

- MCU 型号为 STM32F407xx；
- 使用正确的 ST-Link/SWD 接线：SWDIO、SWCLK、GND，以及目标板供电参考电压；
- 先只连接主控供电和 ST-Link，确认可以擦写和复位；
- 再逐步接入 CAN 收发器、遥控器、舵机和电机驱动电源。

## 3. 接线与外设核对

| 外设 | 当前配置 | 用途 |
| --- | --- | --- |
| CAN1 | 1 Mbps，RX FIFO0 中断 | M3508 ×2、M2006 ×2、GM6020 ×1 |
| CAN2 | 1 Mbps，RX FIFO0 中断 | DM4310 ×2 |
| USART3 RX | PC11，DMA1 Stream1 | 遥控器数据接收 |
| TIM1 CH1/CH2/CH3 | PE9 / PE11 / PE13，50 Hz | Pitch、升降、夹爪舵机 |
| TIM6 | 2 ms 更新中断 | 整车控制周期 |

CAN 总线两端必须各有一个 120 Ω 终端电阻，且所有参与设备必须共地。CAN_H/CAN_L 接反或缺少终端电阻会导致无反馈或间歇性丢帧。

## 4. 推荐上电顺序

1. 断开电机功率电源，仅给主控板和调试器供电；
2. 烧录程序并确认 MCU 未复位循环；
3. 接入遥控器，观察是否持续收到有效帧；
4. 接入 CAN1，确认各 DJI 电机反馈 ID；
5. 接入 CAN2，确认两个 DM4310 反馈与 CAN ID；
6. 让履带悬空后接通电机功率，先保持遥控右拨杆在 `UP`（待机）；
7. 确认方向和限位后，再进入低速运动测试。

## 5. 通信调试

### 遥控器

`Remote_UpdateOnlineStatus()` 以 300 ms 为超时阈值。可在调试器中观察：

```c
Remote_GetData()->IsOnline
Remote_GetData()->LastUpdateTimeMS
Remote_GetData()->RightH
Remote_GetData()->RightV
```

当前代码使用 **18 字节帧长**，但解析逻辑以 `0x0F` 帧头和第 17 字节的开关位读取数据。这与常见 DJI DR16 的 DBUS（18 字节、常用 100 kbit/s、8E1）和标准 SBUS（常见 25 字节、100 kbit/s、8E2）并不完全一致。

在连接遥控器前，必须用逻辑分析仪或串口工具确认实际协议、帧长度、奇偶校验和反相需求；随后使 `usart.c` 的串口配置与 `remote.c` 的解析规则保持一致。协议未确认前，不应依赖遥控器数据驱动电机。

### CAN1：DJI 电机

预期反馈 ID：

| 电机 | 反馈 ID | 发送 ID |
| --- | ---: | ---: |
| M3508 左 / 右主履带 | `0x201` / `0x202` | `0x200` |
| M2006 左 / 右副履带 | `0x205` / `0x206` | `0x1FF` |
| GM6020 Yaw（ID 5） | `0x209` | `0x2FF` |

在断开功率、仅连接 CAN 的情况下，确认电调持续发送对应反馈帧。然后观察：

```c
DjiChassisMotor_GetInstance(0)->IsOnline
DjiChassisMotor_GetInstance(1)->IsOnline
DjiYawMotor_GetInstance()->IsOnline
```

### CAN2：DM4310

默认关节 CAN ID：左 `0x01`、右 `0x02`。`Robot_Init()` 会发送使能命令；若电机未进入 MIT 模式，请检查：

- 电机的实际 CAN ID 是否与 `motor_dm.h` 一致；
- 电机型号、固件及 MIT 报文格式是否匹配；
- 反馈的 ID 和字节格式是否与 `DmMotor_UpdateFeedback()` 假设一致；
- 电机错误码 `Feedback.ErrorCode` 是否非零。

## 6. 功能测试顺序

| 阶段 | 遥控输入 | 预期结果 |
| --- | --- | --- |
| 待机 | 右拨杆 UP | 主/副履带速度目标为零；关节进入收折目标 |
| 正常行驶 | 右拨杆 MID，右摇杆 | 主履带差速控制 |
| 爬坡 | 右拨杆 DOWN，右摇杆 | 主履带限速至 1500 RPM |
| 关节模式 | 左拨杆 UP/MID | 平地/坡道关节目标切换 |
| 抓取 | 左拨杆 DOWN | 主履带停止，右摇杆控制 Pitch 和升降，拨轮控制夹爪 |
| Yaw | 非待机时左摇杆水平 | 目标角度按输入累加并限幅 |

首次测试应将摇杆比例、PID 参数和 PWM 行程调低；确认实际机械方向后，再调整代码中的反向符号和目标范围。

## 7. 常见故障定位

| 现象 | 优先检查 |
| --- | --- |
| 固件没有任何控制输出 | `Robot_Init()` 是否执行、TIM6 是否已启动、是否进入 `Error_Handler()` |
| 遥控一直离线 | USART3 的波特率/校验/停止位、协议帧长、DMA/中断、接收引脚与共地 |
| CAN 电机离线 | 电源、CAN_H/L、终端电阻、波特率、CAN ID、接收中断 |
| 电机方向相反 | 在低速悬空状态下调整相应目标速度的正负号；不要先改 PID |
| DM4310 不响应 | MIT 使能帧、CAN ID、驱动器电源、错误码和型号协议 |
| 舵机无动作 | TIM1 PWM 是否已启动、PE9/PE11/PE13 引脚、供电/共地、PWM 极性和脉宽范围 |
| Yaw 上电突动或跨零反向 | 当前实现只使用单圈编码器角度；在实车使用前应增加上电零位对齐、跨圈处理和限位保护 |

## 8. 上车前必须完成的安全项

- 使电机离线成为明确的停机条件，而不仅是记录状态；
- 在遥控离线故障路径中，显式发送 GM6020 的零电压；
- 为 DM4310 和 Yaw 配置机械/软件限位；
- 确认 Yaw 单圈、多圈及零位逻辑；
- 确认遥控协议后再启用运动控制；
- 为关键 CAN 发送失败、CAN 总线错误和电机错误码增加诊断与告警。

## 9. 调试观察点

建议在 IDE 的 Watch 窗口加入以下对象：

```c
Robot_GetStatus()
Remote_GetData()
DjiChassisMotor_GetInstance(0)
DjiChassisMotor_GetInstance(1)
DjiYawMotor_GetInstance()
DmMotor_GetInstance(DM_JOINT_LEFT)
DmMotor_GetInstance(DM_JOINT_RIGHT)
```

如果需要串口日志，建议只记录状态切换、离线事件和 CAN 发送失败；不要在 2 ms 定时器中断中直接使用阻塞式 `printf`。
