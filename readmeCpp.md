# README.md

```markdown
# ROBOT_FIRST 整车控制程序

基于 STM32F407 + FreeRTOS + C++17 的履带式机器人控制程序。

---

## 硬件结构

| 机构 | 电机 | 数量 | 总线 | 控制方式 |
|---|---|:---:|---|---|
| 主履带驱动 | M3508 + C620 | 2 | CAN1 | 速度闭环（电流） |
| 副履带关节 | DM4310 | 2 | CAN2 | MIT 位置控制 |
| 副履带驱动 | M2006 + C610 | 2 | CAN1 | 速度闭环（电流） |
| Yaw 旋转 | GM6020 | 1 | CAN1 | 位置串级 PID（电压） |
| Pitch 舵机 | 标准舵机 | 1 | TIM1 CH1 | PWM 脉宽 |
| 升降舵机 | 标准舵机 | 1 | TIM1 CH2 | PWM 脉宽 |
| 夹爪舵机 | 标准舵机 | 1 | TIM1 CH3 | PWM 脉宽 |

---

## CAN ID 分配

### CAN1（大疆电机）

| 电机 | 拨码 | 反馈 ID | 命令帧 ID | 帧内 Slot |
|---|:---:|:---:|:---:|:---:|
| M3508 左 | 1 | 0x201 | 0x200 | 0 |
| M3508 右 | 2 | 0x202 | 0x200 | 1 |
| M2006 左 | 5 | 0x205 | 0x1FF | 0 |
| M2006 右 | 6 | 0x206 | 0x1FF | 1 |
| GM6020 | 5 | 0x209 | 0x2FF | 0 |

> GM6020 反馈 ID = 0x204 + 拨码，拨码设为 5 时反馈为 0x209，
> 与 M2006（0x205 / 0x206）不冲突。
> 请勿将 GM6020 拨码设为 1～4，否则反馈 ID 与 M3508 / M2006 重叠。

### CAN2（达妙电机）

| 电机 | CAN ID |
|---|:---:|
| DM4310 左关节 | 0x01 |
| DM4310 右关节 | 0x02 |

---

## 遥控器接线与协议

- 接收机：**大疆 DR16**
- 协议：**DBUS**，18 字节，100 Hz
- 接线：DR16 DBUS 引脚 → 板载 Q10 三极管反相 → **PC11（USART3\_RX）**

### USART3 CubeMX 配置

| 参数 | 值 |
|---|---|
| 引脚 | PC11（RX Only） |
| Baud Rate | 100000 |
| Word Length | 9 Bits |
| Parity | Even |
| Stop Bits | 1 |
| Mode | Receive Only |
| DMA | USART3\_RX，Normal，Byte |
| NVIC | USART3 global interrupt 启用 |

> Word Length = 9 Bits + Parity = Even，对应线路实际格式 **8E1**。
> 半传输中断（HT）在 `onUartRxEvent()` 中被主动关闭，
> 防止 DMA 传输至第 9 字节时误触发回调。

### DBUS 帧格式（18 字节）

```
Byte  0~13 : 16 个通道，每通道 11 bit，小端连续排列，从 Byte0 bit0 起
Byte 14    : bit[1:0] = S1（右拨杆），bit[3:2] = S2（左拨杆）
Byte 15~17 : 鼠标 / 键盘数据（本工程未使用）
```

通道值范围：364（最小）/ 1024（中位）/ 1684（最大）

---

## 遥控器控制逻辑

### 右拨杆 S1：主运动状态

| 位置 | 状态 | 说明 |
|:---:|---|---|
| UP | Standby | 待机，所有电机零输出，关节收折至 90° |
| MID | Running | 正常行驶 |
| DOWN | Climb | 爬坡限速，主履带最高 1500 RPM |

### 左拨杆 S2：关节模式

| 位置 | 关节角度 | 说明 |
|:---:|:---:|---|
| UP | 0° | 平地，关节收平 |
| MID | 40° | 斜坡行驶 |
| DOWN | 65° | 进入夹爪模式，底盘停止，舵机可控 |

> 左拨杆拨至 DOWN 时，主运动状态强制切换为 **Grab**，底盘停止行驶。

### 摇杆与拨轮功能

**Running / Climb 模式：**

| 输入 | 功能 |
|---|---|
| 右摇杆 ↑↓ | 主履带前进 / 后退，最高 4000 RPM（Climb 模式限至 1500 RPM） |
| 右摇杆 ←→ | 主履带差速转向，转向增益 0.7 |
| 左摇杆 ↑↓ | 副履带驱动，最高 3000 RPM |
| 左摇杆 ←→ | Yaw 步进旋转，每帧 ±3°，限幅 ±180° |

**Grab 模式（左拨杆 DOWN）：**

| 输入 | 功能 |
|---|---|
| 右摇杆 ↑↓ | 升降舵机步进，每帧 ±20 µs，范围 700～2300 µs |
| 右摇杆 ←→ | Pitch 舵机步进，每帧 ±15 µs，范围 800～2200 µs |
| 拨轮 > +0.5 | 夹爪闭合（1000 µs） |
| 拨轮 < −0.5 | 夹爪张开（2000 µs） |

所有摇杆输入均施加死区（±0.05），归一化至 −1 ～ +1。

---

## 软件架构

### 目录结构

```
Core/Src/
  main.c                    外设初始化（CubeMX 生成），调用 Robot_Init()
  freertos.c                FreeRTOS 任务定义（用户修改区）
  stm32f4xx_it.c            中断入口（CubeMX 生成）
  can.c / usart.c / tim.c … 外设初始化（CubeMX 生成）

User/
  Inc/
    pid.hpp                 通用 PID 控制器
    motor_dji.hpp           大疆电机驱动（M3508 / M2006 / GM6020）
    motor_dm.hpp            达妙电机驱动（DM4310 MIT 模式）
    remote.hpp              DR16 DBUS 遥控器解析
    robot.hpp               整车控制器声明 + C 接口
  Src/
    pid.cpp
    motor_dji.cpp
    motor_dm.cpp
    remote.cpp
    robot.cpp               HAL 回调实现，C 接口实现
```

### 各模块说明

#### pid.hpp / pid.cpp

通用 PID 控制器，支持**位置式**和**增量式**两种模式，由 `PidMode` 枚举选择。

```
Pid::init(cfg)              以 PidConfig 初始化，清零所有历史状态
Pid::update(setpoint, meas) 输入目标值与测量值，返回本次输出
Pid::reset()                清零误差、积分、输出，不改变参数
Pid::output()               获取上次 update() 的输出值
```

位置式（`PidMode::Position`）计算过程：

$$
u = K_p \cdot e + \sum K_i \cdot e + K_d \cdot (e - e_{prev})
$$

增量式（`PidMode::Increment`）计算过程：

$$
\Delta u = K_p(e - e_{prev}) + K_i \cdot e + K_d(e - 2e_{prev} + e_{prev2})
$$

积分项独立限幅（`integralLimit`），总输出独立限幅（`outputLimit`），
`clamp()` 在限幅值为 0 时直接透传，便于关闭某一路限幅。

---

#### motor_dji.hpp / motor_dji.cpp

大疆电机驱动，包含三个独立控制器类。

**`DjiMotorInstance`** — 单电机数据与状态管理

```
init(feedbackId, commandId, slot, type, speedCfg, posCfg)
    初始化 ID、类型、PID，posCfg 为 nullptr 时跳过位置环初始化

updateFeedback(pData)
    解析 8 字节 CAN 反馈帧，更新 angleRaw / speedRpm / torqueCurrent / temperature
    同时维护多圈累计角度 angleDeg_：
      首次调用以当前编码器值为零点
      后续通过 delta 与 ±4096 阈值判断是否跨圈，累计 roundCount_

updateOnlineStatus(nowMs)
    超过 kDjiOfflineTimeoutMs（100 ms）未收到反馈则标记离线

resetAngle()
    下次调用 updateFeedback() 时重新以当前位置为零点
```

`speedPid` 和 `posPid` 为公有成员，供控制器类直接访问。

**`DjiSendCanFrame`** — CAN 帧发送工具函数

将最多 4 路 `int16_t` 输出值打包为 8 字节标准帧并通过 HAL 发送，
发送前检查发送邮箱是否空闲，邮箱满时返回 `false`。

**`ChassisMotorController`**（M3508 主履带）

```
init(hcan)                          初始化两个 M3508 实例，配置速度环 PID
setTargetSpeedRpm(idx, rpm)         设置目标转速
sendAllCurrent()                    PID 计算 → 限幅 → 发送 0x200 帧
updateFeedback(canId, pData)        按反馈 ID 分发到对应电机实例
updateOnlineStatus()                更新两路在线状态
isAllOnline()                       两路均在线时返回 true
getInstance(idx)                    获取只读电机实例指针
```

**`SubTrackMotorController`**（M2006 副履带驱动）

结构与 `ChassisMotorController` 相同，命令帧为 0x1FF，
发送时固定填充 4 个 slot（M2006 占 slot 0/1，slot 2/3 填零）。

**`YawMotorController`**（GM6020 Yaw 旋转）

```
init(hcan)                          初始化位置外环 + 速度内环 PID
setTargetAngleDeg(angleDeg)         设置目标累计角度（支持多圈）
syncTargetToCurrent()               目标同步至当前实际角度，同时重置两环 PID
                                    用于遥控器恢复时消除跳变
sendVoltage()                       运行串级 PID，发送 0x2FF 帧
```

串级 PID 执行顺序（`runCascadePid()`）：

```
位置外环：posPid.update(targetAngleDeg_, motor_.angleDeg()) → 目标 RPM
速度内环：speedPid.update(目标 RPM, feedback().speedRpm)   → 电压值
```

---

#### motor_dm.hpp / motor_dm.cpp

达妙 DM4310 MIT 模式驱动。

**MIT 命令帧格式（8 字节）：**

```
Byte 0~1 : pos[15:0]   位置（16 bit）
Byte 2   : vel[11:4]
Byte 3   : vel[3:0] | kp[11:8]
Byte 4   : kp[7:0]
Byte 5   : kd[11:4]
Byte 6   : kd[3:0] | tor[11:8]
Byte 7   : tor[7:0]
```

**MIT 反馈帧格式（8 字节）：**

```
Byte 0        : [7:4] ErrorCode  [3:0] MotorID
Byte 1~2      : pos[15:0]
Byte 3[7:4]   : vel[11:8]
Byte 3[3:0]   : vel[7:4]（注：文档中 vel 共 12 bit，跨 Byte3/4）
Byte 4[7:4]   : vel[3:0]
Byte 4[3:0]   : tor[11:8]
Byte 5        : tor[7:0]
Byte 6        : MOS 温度
Byte 7        : 线圈温度
```

定点编解码均使用线性映射：

$$
\text{raw} = \frac{v - v_{min}}{v_{max} - v_{min}} \times (2^{bits} - 1)
$$

**`DmMotorInstance`** — 单电机数据与状态

```
init(canId, kp, kd)         设置 CAN ID 与默认 MIT 参数
parseFeedback(pData)        解析反馈帧，更新 angleDeg / velocityDegPerS / torque
updateOnlineStatus(nowMs)   超过 200 ms 未收到反馈则标记离线
```

**`DmMotorController`**

```
init(hcan)                              初始化左右两个 DM4310 实例（CAN2）
enable(idx) / disable(idx)             发送使能 / 失能特殊帧（0xFC / 0xFD）
clearFault(idx)                        发送清除故障帧（0xFB）
setMitTarget(idx, pos, vel, tor, kp, kd)  设置 MIT 目标参数
sendMitCommand(idx)                    编码并发送 MIT 命令帧
updateFeedback(canId, pData)           按 CAN ID 分发反馈，更新在线时间戳
updateOnlineStatus()                   检查两路离线状态
```

特殊命令帧格式：Byte0～6 填 0xFF，Byte7 为命令字节（0xFB / 0xFC / 0xFD）。

---

#### remote.hpp / remote.cpp

大疆 DR16 DBUS 遥控器解析。

```
RemoteReceiver::init(huart)
    保存 UART 句柄，启动 DMA + IDLE 接收（HAL_UARTEx_ReceiveToIdle_DMA）

RemoteReceiver::parseDbus(pData, len)
    len < 18 时丢弃，≥18 时解析：
      extractChannel(pData, ch)  从连续 bit 流中提取指定通道的 11 bit 值
      toNormalized(raw)          [364,1684] → 施加死区 → 归一化至 [-1,1]
    解析 Byte14 得到 S1 / S2 开关位置（SwitchPos::Up/Mid/Down）
    更新 data_.online 与 data_.lastUpdateMs

RemoteReceiver::updateOnlineStatus()
    超过 300 ms 未收到数据则 data_.online = false
```

`extractChannel()` 实现：

```
bitOffset  = channelIndex × 11
byteOffset = bitOffset / 8
bitShift   = bitOffset % 8
raw = (pData[byteOffset] | pData[byteOffset+1]<<8 | pData[byteOffset+2]<<16)
result = (raw >> bitShift) & 0x7FF
```

---

#### robot.hpp / robot.cpp

整车控制主模块，持有所有子模块实例，实现状态机与指令解算。

**C 接口（供 freertos.c 调用）：**

```c
void Robot_Init(void);   // 调用 g_robot.init()
void Robot_Task(void);   // 调用 g_robot.task()
```

**HAL 回调（覆盖 HAL 弱函数）：**

```
HAL_CAN_RxFifo0MsgPendingCallback(hcan)
    读取 CAN 帧，按 hcan 实例分发至 can1RxDispatch() 或 can2RxDispatch()

HAL_UARTEx_RxEventCallback(huart, size)
    转发至 g_robot.onUartRxEvent()
```

**`RobotController::init()`**

1. 从 `extern` 获取所有 HAL 句柄（hcan1 / hcan2 / htim1 / huart3）
2. 依次初始化 chassis / subTrack / yaw / joint / remote
3. 配置 CAN 过滤器（ID 掩码全通），启动 CAN1 / CAN2，使能 FIFO0 中断
4. 清除 DM4310 故障，延时 10 ms 后发送使能命令
5. 启动三路 TIM1 PWM，设置初始舵机脉宽

**`RobotController::task()`**（每 2 ms 调用）

```
1. checkOnlineStatus()      遥控器 / 底盘在线状态更新
2. [遥控器离线] faultStop() 故障停车并 return
3. updateStateMachine()     拨杆 → RobotState / JointMode
4. resolveChassisCmd()      右摇杆 → 左右轮目标 RPM
5. resolveJointCmd()        关节模式 → DM4310 目标角度
6. resolveYawCmd()          左摇杆水平 → Yaw 累计目标角度
7. resolveServoCmd()        摇杆 / 拨轮 → 三路舵机脉宽（仅 Grab 模式）
8. applyChassisMotors()     M3508 + M2006 PID 计算并发送 CAN 帧
9. applyJointMotors()       DM4310 发送 MIT 命令帧
10. applyYawMotor()         GM6020 串级 PID 并发送 CAN 帧
11. applyServos()           更新三路 TIM1 比较值
```

**`faultStop()`**（遥控器离线时调用）

- 主履带 / 副履带电流清零并发送
- `yaw_.syncTargetToCurrent()` 同步目标，防止恢复时跳变
- DM4310 切换为纯阻尼（Kp = 0，Kd = 2.0），目标位置锁定为当前反馈角度
- 状态强制回到 Standby

**`resolveChassisCmd()`**

```
fwd  = applyDeadband(rightV, 0.05)
turn = applyDeadband(rightH, 0.05)
scale = Climb 模式 ? (1500/4000) : 1.0

leftRpm  = (fwd + turn × 0.7) × 4000 × scale
rightRpm = (fwd - turn × 0.7) × 4000 × scale
```

Standby / Grab 模式下直接输出零。

**`applyChassisMotors()`**

M3508 右侧电机取反（修正安装方向）；
M2006 副履带右侧同样取反，转速由左摇杆垂直轴控制，与主状态无关。

**`setServoPulse(channel, pulseUs)`**

调用前先通过 `clampPulse()` 将脉宽限制在 500～2500 µs，
再写入 `__HAL_TIM_SET_COMPARE()`，要求 TIM1 预分频使计数单位为 1 µs。

---

## FreeRTOS 配置

| 项目 | 值 |
|---|---|
| Tick Rate | 1000 Hz（1 Tick = 1 ms） |
| OS 接口 | CMSIS-RTOS v2 |
| HAL 时基 | TIM7（不使用 SysTick，避免与 RTOS 冲突） |
| RobotTask 优先级 | AboveNormal |
| RobotTask 栈大小 | 2048 Bytes（按实际水位调整） |
| CAN RX 中断优先级 | 5 |
| USART3 中断优先级 | 6 |

> 所有在中断中间接使用 FreeRTOS API 的中断，优先级数值须
> ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（通常为 5），
> 否则触发 `configASSERT` 硬件异常。

`StartRobotTask` 使用 `osDelayUntil()` 保证严格 2 ms 周期，
若任务超时则重置节拍基准，避免连续追赶导致 CPU 占满。

---

## PID 参数

| 电机 | 控制环 | Kp | Ki | Kd | 输出限幅 |
|---|---|---:|---:|---:|---|
| M3508 | 速度环 | 10.0 | 0.5 | 0.0 | 16384（电流值） |
| M2006 | 速度环 | 8.0 | 0.3 | 0.0 | 10000（电流值） |
| GM6020 | 位置外环 | 8.0 | 0.0 | 0.1 | 200 RPM |
| GM6020 | 速度内环 | 100.0 | 2.0 | 0.0 | 30000（电压值） |

DM4310 MIT 参数：

| 模式 | Kp | Kd |
|---|---:|---:|
| 正常（Flat / Ramp / Step） | 15.0 | 1.5 |
| 故障保护（阻尼保持） | 0.0 | 2.0 |

以上均为初始调试值，需根据实际机械结构重新整定。

---

## DM4310 配置

在达妙上位机中写入以下参数后再上电：

| 参数 | 要求值 |
|---|---|
| 工作模式 | MIT |
| CAN 波特率 | 1 Mbps |
| 左关节 CAN ID | 0x01 |
| 右关节 CAN ID | 0x02 |
| 位置范围 | ±360° |
| 速度范围 | ±2000 °/s |
| 力矩范围 | ±10 Nm |

> 若固件版本导致物理范围不同，须同步修改 `motor_dm.hpp` 中的
> `kDm4310PosMaxDeg` / `kDm4310VelMaxDegPerS` / `kDm4310TorqueMax`，
> 否则 MIT 帧定点编解码将产生系统性偏差。

---

## 编译环境

| 工具 | 要求 |
|---|---|
| STM32CubeCLT | 1.21.0 或更高 |
| arm-none-eabi-gcc | 随 CubeCLT 附带 |
| CMake | 随 CubeCLT 附带 |
| C 标准 | gnu11 |
| C++ 标准 | C++17 |
| RTOS | FreeRTOS（CMSIS-RTOS v2） |

`CMakeLists.txt` 必要修改：

```cmake
project(ROBOT_FIRST LANGUAGES C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 将 User/Src/*.cpp 加入源文件列表
# 将 User/Inc 加入头文件搜索路径
```

---

## 首次上电调试

> 所有步骤在**悬空、无负载**状态下进行，确认无误后再安装机械结构。

1. **硬件检查**
   - CAN1 / CAN2 各接 120 Ω 终端电阻
   - DR16 接收机供电正常，与遥控器对频完成
   - 所有电调 / 电机 CAN 拨码与上表一致

2. **达妙上位机**
   - 用 USB 逐一连接 DM4310，写入 CAN ID 与 MIT 模式

3. **上电观测**（Ozone / 串口打印）
   - `g_robot.isRemoteOnline()` 应为 `true`
   - `remote_.data().rightV` 应随右摇杆上下变化（−1 ～ +1）
   - 各电机静止时反馈 RPM 接近 0

4. **主履带测试**
   - 右拨杆 MID → Running，缓慢推右摇杆
   - 若方向相反，在 `applyChassisMotors()` 中修改取反逻辑

5. **副履带关节测试**
   - 拨动左拨杆 UP / MID / DOWN，观察 DM4310 角度变化
   - 确认左右关节运动方向与机械设计一致

6. **Yaw 测试**
   - Running 模式下推左摇杆水平轴，确认 GM6020 旋转方向及限幅

7. **夹爪测试**
   - 左拨杆 DOWN → Grab，推右摇杆控制升降和 Pitch，转动拨轮开合夹爪

---

## 已知限制

- **GM6020 零点**：基于上电后编码器累计值，断电重启后零点重置，不具备掉电记忆。
- **DM4310 反馈解析**：字节布局以本工程代码为准，不同固件版本需对照达妙手册核对。
- **无键鼠支持**：全部输入来自 DR16 遥控器，Byte15～17 未解析。
- **副履带驱动方向**：M2006 左右取反在 `applyChassisMotors()` 中硬编码，安装方向不同时需修改。
- **舵机分辨率**：需在 CubeMX 中确认 TIM1 预分频使计数单位为 1 µs，否则脉宽步进值失准。
```