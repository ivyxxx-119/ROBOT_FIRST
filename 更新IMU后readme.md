下面这份可以直接保存为根目录 **`README.md`**。内容以你这次提供的实现为主，侧重架构、调用链、硬件分配、控制逻辑和调试变量，不包含构建命令、烧录步骤或现场操作流程。

**说明：这次没有提供 `robot.hpp`、`motor_dji.hpp`、`motor_dm.hpp`、`remote.hpp` 和外设初始化文件，因此仅在头文件中定义的数值、DM 收发 ID、GM6020 命令 ID、具体引脚与波特率，不虚构成已确认配置。**

---

# ROBOT_FIRST 工程架构与代码说明

基于 **STM32F407、STM32 HAL、FreeRTOS 和 C++17** 的履带机器人控制固件。

工程将遥控输入、IMU 数据和电机反馈转换为履带、关节、升降、Yaw 和舵机的目标，再通过电机闭环和 CAN/PWM 接口输出。

本文描述当前代码的：

- 工程分层和模块职责；
- 硬件与通信资源分配；
- 初始化及周期控制调用关系；
- 状态机和目标解算；
- 电机协议及闭环控制；
- IMU 服务；
- 调试变量及其实际含义；
- 已实现逻辑与尚未形成保护的边界。

**本文描述软件实现，不将注释、目标指令或在线标志视为硬件动作已完成的证明。**

## 1. 工程目录与分层

```text
User/
├── Application/
│   ├── robot.cpp / robot.hpp
│   ├── robot_api.cpp / robot_api.hpp
│   ├── chassis.cpp / chassis.hpp
│   ├── manipulator.cpp / manipulator.hpp
│   └── communication.cpp / communication.hpp
│
├── Service/
│   └── imu_service.cpp / imu_service.hpp
│
├── Driver/
│   ├── Sensor/
│   │   └── bmi088.cpp / bmi088.hpp
│   ├── Motor/
│   │   ├── motor_dji.cpp / motor_dji.hpp
│   │   └── motor_dm.cpp / motor_dm.hpp
│   └── Remote/
│       └── remote.cpp / remote.hpp
│
├── Bsp/
│   ├── bsp_bmi088.cpp / bsp_bmi088.hpp
│   └── bsp_delay.cpp / bsp_delay.hpp
│
├── Algorithm/
│   └── pid.cpp / pid.hpp
│
└── Debug/
    └── debug.cpp / debug.hpp
```

CubeMX/HAL/FreeRTOS 相关代码位于：

```text
Core/
Drivers/
Middlewares/
```

### 1.1 各层职责

| 层级        | 职责                                           | 主要模块                                           |
| ----------- | ---------------------------------------------- | -------------------------------------------------- |
| Application | 整车状态机、模式切换、目标解算、执行器输出编排 | `robot`、`chassis`、`manipulator`、`communication` |
| Service     | 采样数据处理、校准、滤波和有效性管理           | `imu_service`                                      |
| Driver      | 设备协议、反馈解析、设备状态和控制接口         | `motor_dji`、`motor_dm`、`remote`、`bmi088`        |
| BSP         | 板卡资源绑定、具体 HAL 外设操作和平台延时      | `bsp_bmi088`、`bsp_delay`                          |
| Algorithm   | 可复用控制算法                                 | PID                                                |
| Debug       | 调试观察与诊断变量                             | `debug`                                            |

当前电机 Driver 还包含速度环、位置环等控制职责。它并非仅负责报文编解码，也尚未完全通过 BSP 隔离 HAL。

BMI088 的分层更明确：

```text
Application
    ↓ 是否使用 IMU 数据辅助控制
imu_service
    ↓ 校准、滤波、坐标转换、有效性
bmi088
    ↓ 寄存器协议、初始化、物理量换算
bsp_bmi088
    ↓ SPI1、片选 GPIO、HAL 传输
HAL / bsp_delay
```

## 2. 整车对象与应用层组织

工程由唯一的整车对象管理控制状态：

```cpp
namespace
{
    Robot::RobotController g_robot;
}
```

该对象定义在 `robot_api.cpp` 的匿名命名空间中，外部通过接口访问：

```cpp
void Robot_Init(void);
void Robot_Task(void);
```

`RobotController` 持有或管理以下对象：

```text
chassis_     → 两台主履带 M3508
subTrack_    → 两台副履带 M2006
lift_        → 一台升降 M2006
yaw_         → 一台 GM6020
joint_       → 两台 DM4310
remote_      → DBUS 接收器
```

### 2.1 文件职责

| 文件                | 主要职责                                                     |
| ------------------- | ------------------------------------------------------------ |
| `robot.hpp`         | 状态枚举、控制参数、目标结构、控制对象及方法声明             |
| `robot.cpp`         | 初始化、任务主流程、状态机、模式进入动作、在线状态和失联处理 |
| `chassis.cpp`       | 差速解算、主副履带方向处理、IMU 辅助、M2006 合并输出         |
| `manipulator.cpp`   | 关节、左摇杆仲裁、Yaw、升降、夹爪和舵机输出                  |
| `communication.cpp` | CAN 启动、接收分发、UART 接收事件和遥控解析衔接              |
| `robot_api.cpp`     | 全局对象、C 接口、HAL 回调桥接                               |
| `robot_api.hpp`     | 面向 C/C++ 的公开调用入口                                    |

这些应用层 `.cpp` 主要实现同一个 `RobotController` 类，**不是多个独立任务**。

## 3. 硬件与通信资源分配

### 3.1 执行器分配

| 机构       | 执行器      | 数量 | MCU 接口                | 控制对象                    |
| ---------- | ----------- | ---: | ----------------------- | --------------------------- |
| 主履带     | M3508       |    2 | CAN1                    | `chassis_`                  |
| 副履带驱动 | M2006       |    2 | CAN2                    | `subTrack_`                 |
| 升降       | M2006       |    1 | CAN2                    | `lift_`                     |
| Yaw        | GM6020      |    1 | CAN1                    | `yaw_`                      |
| 副履带关节 | DM4310      |    2 | CAN2                    | `joint_`                    |
| 折叠舵机   | PWM 舵机    |    1 | TIM1，`kServoFoldCh`    | `servoCmd_.foldUs`          |
| 夹爪舵机   | PWM 舵机    |    1 | TIM1，`kServoGripperCh` | `servoCmd_.gripperUs`       |
| 遥控接收机 | DBUS 接收机 |    1 | USART3 + DMA/IDLE       | `remote_`                   |
| IMU        | BMI088      |    1 | SPI1 + 两路片选         | BMI088 Driver / IMU Service |

M2006 总数为三台，升降与副履带是独立电机实例。

舵机初始化只启动两个通道。现有模块定义对应 TIM1 CH2、CH3，实际绑定以 `manipulator.hpp` 为准。

### 3.2 DJI CAN 分组

| 总线 | 电机           | 反馈 ID          | 命令 ID                                  |       Slot |
| ---- | -------------- | ---------------- | ---------------------------------------- | ---------: |
| CAN1 | 左 M3508       | `0x201`          | `kChassisCmdId`，对应主履带 `0x200` 分组 |          0 |
| CAN1 | 右 M3508       | `0x202`          | 同上                                     |          1 |
| CAN1 | GM6020         | `kYawFeedbackId` | `kYawCmdId`                              | `kYawSlot` |
| CAN2 | 左副履带 M2006 | `0x205`          | `0x1FF`                                  |          0 |
| CAN2 | 右副履带 M2006 | `0x206`          | `0x1FF`                                  |          1 |
| CAN2 | 升降 M2006     | `0x207`          | `0x1FF`                                  |          2 |

GM6020 的 ID、Slot 和电流控制配置必须以最新 `motor_dji.hpp` 及电机实际配置为准，不能沿用旧文档中的电压控制分组推断。

三台 M2006 在应用层合并为一帧：

```text
CAN2 / 0x1FF / 8 Bytes

Bytes 0–1：左副履带输出
Bytes 2–3：右副履带输出
Bytes 4–5：升降输出
Bytes 6–7：0
```

每路输出是高字节在前的 16 位控制值。

### 3.3 DM4310 ID 与编码范围

DM 驱动分别保存发送和反馈 ID：

```cpp
kDmJointLeftTxCanId
kDmJointLeftRxCanId

kDmJointRightTxCanId
kDmJointRightRxCanId
```

命令 ID 和反馈 ID 不要求相同。

MIT 编码范围由以下常量决定：

```cpp
kDm4310PosMaxRad
kDm4310VelMaxRadPerS
kDm4310TorqueMax
kDm4310KpMax
kDm4310KdMax
```

这些范围必须与电机固件配置一致。应用层目标使用度、度/秒，驱动在发送前转换为弧度、弧度/秒。

### 3.4 外设配置边界

当前应用代码使用以下 HAL 资源：

```cpp
hcan1
hcan2
htim1
huart3
```

BMI088 BSP 使用 SPI1 和两个片选引脚。

具体引脚、CAN 波特率、SPI 极性/相位、DMA Stream、外设中断优先级和 PWM 周期，由 `.ioc` 与 `Core/Src` 的外设初始化代码决定，不能仅从应用层确定。

舵机代码直接写入：

```cpp
__HAL_TIM_SET_COMPARE(htim1_, channel, pulseUs);
```

因此只有计数单位为 \(1\ \mu s\) 时，`pulseUs` 才能直接表示微秒脉宽。

## 4. 初始化调用关系

```text
main.c
    ↓ Robot_Init()
robot_api.cpp
    ↓ g_robot.init()
RobotController::init()
```

`init()` 的实际顺序：

```text
Imu_Init()
    ↓
绑定 CAN1 / CAN2 / TIM1 / USART3 句柄
    ↓
设置状态、机构状态和初始目标
    ↓
初始化 chassis / subTrack / lift / yaw / joint / remote
    ↓
startCan()
    ↓
HAL_Delay(1000 ms)
    ↓
清除左右 DM 故障
    ↓
HAL_Delay(20 ms)
    ↓
发送左右 DM 使能命令
    ↓
启动折叠和夹爪 PWM
    ↓
写入初始舵机目标
    ↓
进入 Standby
```

初始化前必须完成必要的时钟、GPIO、SPI、CAN、UART、TIM、DMA 和延时初始化。

`g_robot` 的静态构造阶段只应设置软件初值，不应访问尚未初始化的硬件。

## 5. 周期任务调用关系

```text
FreeRTOS 任务循环
    ↓ Robot_Task()
robot_api.cpp
    ↓ g_robot.task()
RobotController::task()
```

每次 `task()` 按以下顺序执行：

```text
1. ++debugTaskCount
2. Imu_Update()
3. checkOnlineStatus()
4. 更新遥控调试信息

5. 遥控离线？
   ├─ 是：faultStop() → 清除辅助调试状态 → return
   └─ 否：继续

6. updateStateMachine()
7. 状态改变时调用 onStateEnter()
8. 保存 prevState_ 和调试状态

9. resolveChassisCmd()
10. resolveJointCmd()
11. resolveLeftStickAxis()
12. resolveYawCmd()
13. resolveLiftCmd()
14. resolveServoCmd()

15. applyChassisMotors()
16. applyJointMotors()
17. applyYawMotor()
18. applyLiftMotor()
19. applyServos()
```

`Robot_Task()` 本身不创建任务，也不保证执行周期。

关节和升降目标累积使用 `kTaskPeriodS`。Yaw 使用每次调用固定角度步长，因此实际任务频率变化会改变 Yaw 目标变化速度。

## 6. 状态机与模式进入逻辑

### 6.1 状态选择

| 条件                        | 状态      |
| --------------------------- | --------- |
| 初始化阶段                  | `Init`    |
| 右拨杆 Up                   | `Standby` |
| 右拨杆 Mid                  | `Drive`   |
| 右拨杆 Down                 | `Stair`   |
| 右拨杆非 Up，且抓取请求有效 | `Grab`    |

抓取请求使用边沿检测：

```text
左拨杆：非 Down → Down
    ↓
grabRequested_ = true
```

左拨杆离开 Down 时清除请求。

右拨杆 Up 的分支会：

```cpp
state_ = RobotState::Standby;
grabRequested_ = false;
prevSwitchLeft_ = rc.switchLeft;
```

因此不能只根据左拨杆当前是否为 Down 判断是否进入 Grab。

### 6.2 模式进入动作

| 新状态          | `onStateEnter()` 动作                                                     |
| --------------- | ------------------------------------------------------------------------- |
| Stair           | 从在线左 DM 反馈接管共同目标；否则使用 `kJointFoldDeg`                    |
| Grab            | 升降目标同步当前位置；发出折叠展开目标；同步在线 Yaw 目标                 |
| Standby / Drive | 必要时发出折叠收起目标；设置 `dmTargetDeg_ = kJointFoldDeg`；同步在线 Yaw |
| 其他            | 无额外动作                                                                |

重要实现细节：

- **Grab → Stair 不经过 Standby/Drive 的折叠收起分支**，折叠命令可能保持展开。
- 安全高度判断没有在进入 Grab 时实际执行。
- 状态进入逻辑发出目标，但没有展开/收起到位确认。
- Drive 每轮的关节解算会以实际反馈覆盖关节目标，不能把进入 Drive 时设置收起角等同于持续执行收起运动。

## 7. 履带目标解算与输出

### 7.1 差速混合

`resolveChassisCmd()` 使用：

```cpp
rc.rightV
rc.rightH
```

应用死区后，根据状态选择：

```cpp
kTrackMaxRpm
kTrackClimbRpm
kTrackGrabRpm
```

差速解算：

\[
n_{\mathrm{fwd}}=u_{\mathrm{fwd}}n_{\max}
\]

\[
n_{\mathrm{turn}}=u_{\mathrm{turn}}n_{\max}k_{\mathrm{turn}}
\]

\[
n_L=n_{\mathrm{fwd}}+n_{\mathrm{turn}},\qquad
n_R=n_{\mathrm{fwd}}-n_{\mathrm{turn}}
\]

若峰值超过限幅，对左右目标同时等比例缩放。

Standby 直接设置左右目标为零。

### 7.2 主副履带输出

主履带：

```cpp
左： chassisCmd_.leftRpm
右：-chassisCmd_.rightRpm
```

右侧取反用于补偿安装方向。

副履带：

```cpp
左： chassisCmd_.leftRpm  * kSubTrackRatio
右：-chassisCmd_.rightRpm * kSubTrackRatio
```

`kSubTrackRatio` 表示目标 RPM 跟随比例，不自动保证主副履带地面速度相同。

### 7.3 M2006 合并发送

`applyChassisMotors()` 同时计算：

```cpp
subTrack_.calcOutput();
lift_.calcOutput();
```

随后合并为 `0x1FF` 帧。

Standby 分支：

- 副履带目标设零；
- 清零副履带 PID；
- 副履带两个 Slot 直接发送零；
- 升降仍计算并发送位置控制输出。

`applyLiftMotor()` 当前为空，因为升降输出已经在这里发送，并非漏发。

### 7.4 IMU 直行辅助

当前参数：

```cpp
kBmiStraightAssistEnabled = false;
kBmiStraightKp = 2.0f;
kBmiStraightLimitRatio = 0.05f;
kBmiStraightCorrectionSign = 1.0f;
```

只有同时满足以下条件才产生修正：

```text
开关启用
Drive 状态
转向输入经死区处理后为零
前进输入绝对值大于 0.10
Imu_IsUsable() 为真
```

修正形式：

\[
\Delta n=
\operatorname{clamp}
\left(sK_p\dot{\psi},-0.05n_{\max},0.05n_{\max}\right)
\]

其中 \(\dot{\psi}\) 使用 `bmi_yaw_rate_dps`。

这属于偏航角速度抑制，不是绝对航向保持。当前开关关闭，IMU 更新不会通过这段逻辑改变履带目标。

## 8. 机械机构目标解算

### 8.1 左摇杆仲裁

`resolveLeftStickAxis()` 只在 Grab 生效：

```text
|leftV| ≥ |leftH|：只保留纵向
|leftV| < |leftH|：只保留横向
```

结果保存为：

```cpp
grabLeftVArb_
grabLeftHArb_
```

升降与 Yaw 共用这一仲裁结果，避免斜推时同时变化。

### 8.2 DM 关节

| 状态         | 目标与控制逻辑                                            |
| ------------ | --------------------------------------------------------- |
| Stair        | 左摇杆纵向按时间累积共同目标，限幅，并加入 15% 的速度前馈 |
| Standby      | 固定 `dmTargetDeg_`，使用 `kJointFoldKp/Kd`               |
| Drive / Grab | 在线电机目标逐轮跟随各自反馈；离线时使用保存目标          |

Stair：

\[
\theta_{\mathrm{target}}
\leftarrow
\operatorname{clamp}
\left(
\theta_{\mathrm{target}}+
u\,k_{\mathrm{rate}}\,T,
\theta_{\min},
\theta_{\max}
\right)
\]

发送时左右关节采用相同速度前馈，目标力矩为零。

Drive/Grab 中目标逐轮等于反馈时，位置误差趋近于零。**即使配置了非零 \(K_p\)，也不能将这种逻辑解释为固定位置抗重力保持。**

### 8.3 升降

Grab 中：

```cpp
lift_.updateTargetByJoystick(grabLeftVArb_, kTaskPeriodS);
```

其他模式不更新目标。

驱动使用相对上电零点的编码器累计位置：

\[
p=\frac{\theta_{\mathrm{multi}}}{360^\circ}\times8192
\]

当前目标限幅：

```text
-20000 ～ +20000 count
```

源码明确标注为待实测范围，不是已标定机械行程。

控制结构：

```text
位置误差
    ↓ 比例外环及 RPM 限幅
目标 RPM
    ↓ 速度 PID
电流控制值
    ↓ 叠加 kLiftGravityFF
最终输出
```

位置误差小于 50 count 时：

- 不运行速度 PID；
- 输出固定重力补偿；
- 重置速度 PID。

`isSafeToFold()` 提供位置判断接口，但当前模式切换没有使用它建立互锁。

### 8.4 Yaw

只有 Grab 且电机在线时更新目标：

```cpp
targetYawDeg_ += grabLeftHArb_ * kYawStepDeg;
```

随后限幅至：

```cpp
[-kYawMaxDeg, +kYawMaxDeg]
```

输出逻辑：

| 条件          | 行为                            |
| ------------- | ------------------------------- |
| Standby       | 同步当前位置，发送零电流控制值  |
| 电机离线      | 发送零电流控制值                |
| Grab 且在线   | 设置目标并运行位置—速度串级 PID |
| Drive / Stair | 同步当前位置并发送零电流控制值  |

非 Grab 的“同步目标”用于避免目标残留，**并不意味着执行主动位置锁定**。

Yaw 误差小于 \(1^\circ\) 时，驱动输出零并重置两环 PID。

### 8.5 夹爪和折叠舵机

Grab 中，拨轮超过正负阈值分别设置夹爪闭合、张开目标。

其他状态保留夹爪上次目标。

折叠目标由模式进入逻辑设置，不由连续摇杆控制。

## 9. 电机驱动逻辑

### 9.1 DJI 反馈与累计角度

`DjiMotorInstance::updateFeedback()` 解析：

```text
Bytes 0–1：编码器
Bytes 2–3：RPM
Bytes 4–5：反馈电流
Byte 6：温度
```

第一帧将当前位置作为相对零点。

后续以编码器差值超过 ±4096 判断跨圈，计算累计角度：

\[
\theta=
\frac{e+8192N-e_0}{8192}\times360^\circ
\]

该算法依赖反馈间隔足够短，不能可靠恢复两次采样间跨越过大的转动。它也不提供掉电绝对位置。

### 9.2 当前 PID 初始化参数

| 控制对象     | 环路 | \(K_p\) | \(K_i\) | \(K_d\) | 输出限制            |
| ------------ | ---- | ------: | ------: | ------: | ------------------- |
| M3508        | 速度 |      10 |       0 |       0 | `kM3508MaxCurrent`  |
| 副履带 M2006 | 速度 |       2 |       0 |       0 | `kM2006MaxCurrent`  |
| 升降 M2006   | 速度 |       8 |       0 |       0 | `kM2006MaxCurrent`  |
| GM6020       | 位置 |       8 |       0 |     0.1 | 200 RPM             |
| GM6020       | 速度 |      10 |       0 |       0 | `kGm6020MaxCurrent` |

升降位置比例、最大 RPM 和重力前馈由对应配置成员或常量决定。

DM 实例初始化保存 \(K_p=10\)、\(K_d=1\)，但周期发送前会被应用层 `setMitTarget()` 的参数覆盖。

### 9.3 CAN 发送

DJI 与 DM 发送函数都在邮箱满时忙等，按 HAL tick 判断约 1 ms 超时。

因此：

- 发送不是无等待操作；
- 多次发送可能消耗明显的任务预算；
- HAL tick 停止推进时，超时行为不能按正常情况推断；
- 返回成功表示报文成功加入发送邮箱，不表示电机已经执行。

应用层当前多数发送调用没有检查返回值，没有形成统一发送失败保护。

### 9.4 DM MIT 编解码

发送字段：

```text
位置：16 bit
速度：12 bit
Kp：12 bit
Kd：12 bit
力矩：12 bit
```

线性映射：

\[
q=
\frac{x-x_{\min}}{x_{\max}-x_{\min}}
(2^b-1)
\]

输入先限幅，再编码。

反馈解析：

```text
Byte 0：错误状态 / Motor ID
Bytes 1–2：位置
Byte 3 + Byte 4 高半字节：速度
Byte 4 低半字节 + Byte 5：力矩
```

当前实现没有保存 Byte 6、Byte 7 温度。

特殊命令采用前七字节 `0xFF`、最后一字节命令码的格式。

`enabled_` 在使能报文成功加入邮箱后置位，**不是来自电机反馈的使能确认**。此外，特殊命令接口将 ID 转为 `uint8_t`，配置不能未经修改就使用大于 `0xFF` 的发送 ID。

## 10. 通信接收调用链

### 10.1 CAN

```text
CAN IRQ
    ↓
HAL_CAN_RxFifo0MsgPendingCallback()
    ↓
HAL_CAN_GetRxMessage()
    ↓
CAN1 → can1RxDispatch()
CAN2 → can2RxDispatch()
    ↓
对应控制器 updateFeedback()
    ↓
更新反馈、在线标志和时间戳
```

当前过滤器：

```text
CAN1：Filter Bank 0  → FIFO0
CAN2：Filter Bank 14 → FIFO0
SlaveStartFilterBank：14
掩码：全零
```

过滤较宽，依靠软件按 ID 分发。

FIFO1 回调存在，但 `startCan()` 未启用 FIFO1 通知，也未将当前过滤器分配到 FIFO1。

“CAN1/CAN2 同一物理总线”的注释不是接线证明，FIFO 分配与是否同一物理总线也不是同一概念。

接收边界：

- DM 驱动要求 `DLC == 8`。
- DJI 分发和调试提取未统一检查 DLC。
- 回调直接使用 `StdId`，未统一检查标准帧、数据帧类型。
- 每次回调读取一帧，不是在代码中显式循环排空 FIFO。

### 10.2 遥控器

```text
USART3 DMA / IDLE 事件
    ↓
HAL_UARTEx_RxEventCallback()
    ↓
onUartRxEvent()
    ↓
长度等于 18？
    ├─ 是：parseDbus()
    └─ 否：记录长度拒绝
    ↓
重新启动 ReceiveToIdle DMA
    ↓
关闭 DMA 半传输中断
```

`RemoteReceiver::init()` 的首次接收启动没有在该函数内关闭半传输中断；事件处理后的重启才明确关闭。

### 10.3 当前 DBUS 解码映射

| 字段          | 代码来源            |
| ------------- | ------------------- |
| `rightH`      | ch0                 |
| `rightV`      | ch1                 |
| `leftH`       | ch2                 |
| `leftV`       | ch3                 |
| `dial`        | Bytes 16–17，小端   |
| `switchRight` | Byte 5 bits `[5:4]` |
| `switchLeft`  | Byte 5 bits `[7:6]` |

四个摇杆通道以连续 11 bit 位流提取。

**文件头部对 S1/S2 的名称与变量中的左右命名不能替代物理映射确认。本文以上表中的实际赋值为准。**

解析检查：

- 长度严格等于 18；
- 五个通道在配置范围内；
- 拨杆原始值在 1～3；
- 所有检查通过后才刷新在线时间。

归一化包含原始通道死区和 \([-1,1]\) 限幅。应用层还会再次应用 `kJoystickDeadband`。

接收格式要求与代码注释一致：

```text
100000 baud
8E1
STM32 配置：9 Bits + Even Parity + 1 Stop Bit
```

引脚、DMA 与反相电路由实际硬件配置决定。

## 11. IMU 服务与有效性

IMU 服务的公开接口：

```cpp
void Imu_Init(void);
void Imu_Update(void);
uint8_t Imu_IsUsable(void);
```

职责包括：

```text
BMI088 初始化
静止零偏校准
周期读取
数值合理性检查
车体坐标转换
角速度滤波
低动态加速度倾角参考
数据时效判断
```

BMI088 Driver 输出：

| 数据   | 单位  |
| ------ | ----- |
| 角速度 | rad/s |
| 加速度 | m/s²  |
| 温度   | °C    |

服务层车体角速度显示为度/秒。

`Bmi088_TryRead()` 读取失败时不更新输出。服务层必须检查返回值，不能将保留的旧数据当作新样本。

`Imu_IsUsable()` 同时要求：

```text
初始化成功
零偏校准成功
数据有效
最后成功数据未超时
```

倾角是低动态加速度参考，不是完整融合姿态，也不提供绝对航向。

## 12. 在线状态与失联处理

`checkOnlineStatus()` 更新所有设备在线状态，但：

```cpp
motorAllOnline_ = chassis_.isAllOnline();
```

只汇总主履带，不代表全部执行器在线。

`task()` 当前以遥控离线作为进入 `faultStop()` 的直接条件，没有用 `motorAllOnline_` 阻止全部输出。

### 12.1 `faultStop()` 实际行为

| 对象     | 行为                                         |
| -------- | -------------------------------------------- |
| 主履带   | 零速度目标，继续计算速度 PID 输出            |
| 副履带   | 零速度目标，继续计算速度 PID 输出            |
| 升降     | 保留原目标，继续位置控制与重力前馈           |
| Yaw      | 同步当前位置，发送零电流控制值               |
| DM       | 目标速度零、力矩前馈零、\(K_p=0\)、\(K_d=2\) |
| 舵机     | 不更新目标，硬件继续保留先前 PWM 比较值      |
| 整车状态 | 设为 Standby                                 |

DM 的纯阻尼不等于固定位置保持；目标位置字段在 \(K_p=0\) 时不产生位置刚度。

`faultStop()` 不调用 `onStateEnter(Standby)`，也不显式清除抓取请求。恢复时的模式切换依赖当前拨杆及保留状态。

这些行为属于软件失联策略，不等于所有输出清零或硬件急停。

## 13. 调试变量索引

调试变量反映其**最后一次赋值位置**，并非所有变量都在每轮或每条故障路径中刷新。

### 13.1 主流程与遥控

| 变量                      | 含义                                    |
| ------------------------- | --------------------------------------- |
| `debugTaskCount`          | `task()` 调用次数                       |
| `debugNowMs`              | 周期内读取的 HAL tick                   |
| `debugRemoteAgeMs`        | 当前 tick 与最后有效遥控帧时间差        |
| `debugRemoteOnline`       | 遥控在线状态                            |
| `debugRemoteLastUpdateMs` | 最后有效遥控帧时间                      |
| `debugRobotState`         | `RobotState` 枚举数值，含义以头文件为准 |
| `debugSwitchRightRaw`     | 解析后的右拨杆枚举值                    |
| `debugSwitchLeftRaw`      | 解析后的左拨杆枚举值                    |
| `debugRightV`             | 右摇杆纵向归一化输入                    |
| `debugLeftV`              | Stair 分支中记录的左摇杆纵向输入        |

拨杆和摇杆部分变量只在遥控在线路径更新，离线时可能显示旧值。

### 13.2 UART / DBUS

| 变量                         | 含义                                   |
| ---------------------------- | -------------------------------------- |
| `debugUartRxEventCount`      | UART 接收事件次数                      |
| `debugUartRxSize`            | 最近一次接收长度                       |
| `debugDbusRejectedSizeCount` | 长度不等于 18 的事件次数               |
| `debugDbusAcceptedCount`     | 通过在线标志及时间戳变化推断的接受次数 |

`debugDbusAcceptedCount` 不是严格解析成功计数：两次成功解析若发生在同一个 HAL tick，时间戳相同，可能漏计。

长度正确但通道或拨杆校验失败，也不会增加长度拒绝计数。

### 13.3 主履带

| 变量                                    | 含义                   |
| --------------------------------------- | ---------------------- |
| `debug201RxCount` / `debug202RxCount`   | 对应反馈 ID 的接收次数 |
| `debug201SpeedRpm` / `debug202SpeedRpm` | 最近反馈 RPM           |
| `debugChTx201` / `debugChTx202`         | 计算后准备发送的控制值 |

`debugChTx201/202` 不是发送 ID；两路命令实际打包在主履带命令帧中。变量赋值也不证明发送成功。

### 13.4 CAN2 与 M2006

| 变量                                  | 含义                                   |
| ------------------------------------- | -------------------------------------- |
| `debugCan2LastRxId`                   | 最近 CAN2 接收 ID                      |
| `debugCan2RxCount`                    | CAN2 分发次数                          |
| `debugRx205Count` / `debugRx206Count` | 两台副履带反馈次数                     |
| `debugAngle205`                       | 左副履带原始编码器值                   |
| `debugSpeed205`                       | 左副履带反馈 RPM                       |
| `debugSubTargetRpm0/1`                | 驱动中的左右副履带目标 RPM，右侧已取反 |
| `debugSubFeedRpm0/1`                  | 左右副履带反馈 RPM                     |
| `debugSubOutput0/1`                   | 左右副履带计算输出                     |
| `debugOut205`                         | 合并帧 Slot 0                          |
| `debugOut206`                         | 合并帧 Slot 1                          |
| `debugOut207`                         | 合并帧 Slot 2                          |

Standby 提前返回分支会更新 `debugOut205/206`，但没有同步更新 `debugOut207` 和副履带详细变量。

失联路径也没有完整刷新这些变量，因此可能显示之前的正常周期值。

### 13.5 DM 关节

| 变量                                             | 含义                                      |
| ------------------------------------------------ | ----------------------------------------- |
| `debugDmRxLeftCount` / `debugDmRxRightCount`     | 匹配 DM 反馈 ID 的次数，尚未保证 DLC 合格 |
| `debugDmTargetDeg`                               | Stair 中累积的共同目标                    |
| `debugDmLeftOnline` / `debugDmRightOnline`       | 最近正常输出阶段记录的在线状态            |
| `debugDmLeftError` / `debugDmRightError`         | 反馈状态字段                              |
| `debugDmLeftAngleDeg` / `debugDmRightAngleDeg`   | 反馈位置，度                              |
| `debugDmLeftVelDps` / `debugDmRightVelDps`       | 反馈速度，度/秒                           |
| `debugDmLeftTorque` / `debugDmRightTorque`       | 解码力矩                                  |
| `debugDmLeftTargetDeg` / `debugDmRightTargetDeg` | 最近正常输出阶段的命令目标                |

失联时不经过 `applyJointMotors()`，这些变量不保证显示失联阻尼命令。

反馈状态字段具体取值含义应对照电机固件手册，不能简单认定所有非零值都是同一种故障。

### 13.6 IMU 与直行辅助

| 变量组                                                        | 含义                     |
| ------------------------------------------------------------- | ------------------------ |
| `bmi_test_init_error`、`bmi_test_init_attempts`               | 初始化结果和尝试次数     |
| `bmi_test_read_count`                                         | 服务周期成功读取次数     |
| `bmi_test_ready`、`bmi_test_bias_ready`                       | 初始化、零偏校准状态     |
| `bmi_test_data_valid`、`bmi_test_tilt_valid`                  | 数据与倾角可信标志       |
| `bmi_test_last_update_ms`                                     | 最后合理有效样本时间     |
| `bmi_test_gyro[3]`、`bmi_test_accel[3]`、`bmi_test_temp`      | Driver 原始物理量输出    |
| `bmi_body_gyro[3]`、`bmi_body_accel[3]`                       | 车体坐标数据             |
| `bmi_roll_rate_dps`、`bmi_pitch_rate_dps`、`bmi_yaw_rate_dps` | 滤波角速度               |
| `bmi_pitch_deg`、`bmi_roll_deg`                               | 低动态倾角参考           |
| `debugBmiYawRateDps`                                          | 应用层记录的偏航角速度   |
| `debugBmiStraightCorrectionRpm`                               | 本轮辅助差速修正         |
| `debugBmiStraightActive`                                      | 本轮是否进入辅助修正分支 |

当前辅助关闭时：

```text
debugBmiStraightActive = 0
debugBmiStraightCorrectionRpm = 0
```

并不表示 IMU 没有更新。

BMI088 芯片 ID、子芯片初始化结果和 BSP SPI 错误查询属于传感器诊断，名称与定义以 `debug.hpp` 和 BSP 接口为准。

## 14. 并发与实现边界

### 14.1 中断与任务共享状态

CAN/UART 回调更新驱动状态，整车任务同时读取这些状态。

当前代码没有展示锁、临界区或快照机制。即使单个字段访问在 MCU 上可原子完成，也不能保证：

- 多字段遥控数据是一致快照；
- 编码器、累计角度、速度和时间戳来自同一次反馈；
- 调试器读取的一组变量属于同一周期。

`volatile` 主要影响访问优化，不提供完整的线程同步或结构体一致性。

### 14.2 当前未形成的统一保护

以下事项不能视为已完整实现：

- 全部电机在线汇总及输出联锁；
- CAN 发送失败统一处理；
- DM 状态统一故障处理；
- 折叠安全高度和到位互锁；
- 升降绝对回零及已标定硬限位；
- 反馈帧类型、DLC 的统一验证；
- 失联恢复时的统一请求复位；
- 所有故障路径中的调试状态完整刷新。

### 14.3 代码行为与名称的区别

- `foldState_` 是软件命令状态，不是舵机实际位置反馈。
- `enabled_` 是软件使能发送状态，不是电机使能确认。
- `motorAllOnline_` 当前只代表两台主履带。
- `sendZeroCurrent()` 是零控制值，不是机械锁定。
- `faultStop()` 是分类失联策略，不是全部执行器断电。
- `applyLiftMotor()` 为空，是因为升降已合并发送。
- CAN 发送返回成功，是加入发送邮箱，不是执行完成。
- 调试输出变量是准备发送的命令，不是总线发送成功证明。

## 15. 核心调用关系汇总

```text
                         main.c
                            │
                       Robot_Init()
                            │
                      robot_api.cpp
                            │
                       g_robot.init()
                            │
          ┌─────────────────┼──────────────────┐
          ▼                 ▼                  ▼
       Imu_Init()      设备对象初始化       CAN / PWM 启动


                  FreeRTOS 周期任务
                            │
                       Robot_Task()
                            │
                       g_robot.task()
                            │
          ┌─────────────────┼──────────────────┐
          ▼                 ▼                  ▼
      Imu_Update()      在线/状态机        目标解算与输出
                                             │
                       ┌─────────────────────┼───────────────────┐
                       ▼                     ▼                   ▼
                  chassis.cpp        manipulator.cpp       电机控制器
                       │                     │                   │
                       └─────────────────────┴───────────────────┘
                                             │
                                         CAN / PWM


 CAN / UART 中断
       │
       ▼
 robot_api.cpp 的 HAL 回调
       │
       ▼
 communication.cpp
       │
       ├─ CAN1 → 主履带 / Yaw
       ├─ CAN2 → 副履带 / 升降 / DM
       └─ UART → DBUS 解析
                     │
                     ▼
              更新设备数据和时间戳
                     │
                     └────────→ 下一轮整车控制
```

**整体控制闭环为：输入与反馈 → 状态判断 → 目标解算 → 控制计算 → 通信输出 → 设备反馈。应用层决定“整车如何动作”，设备驱动负责“目标如何变成报文和控制输出”，IMU 服务负责“传感器数据是否适合使用”。**