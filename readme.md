# ROBOT_FIRST（C++ 版本）

STM32F407 履带机器人控制固件。系统采用 C++ 对象化架构：遥控输入、PID、电机总线和整车控制器各自管理状态。STM32 HAL 回调通过 C 链接桥接到对象方法。

> 首次上电请使履带悬空、关节无负载，并准备物理急停。协议、旋转方向、限位和 PID 未确认前不得落地运行。

## 硬件功能

| 模块 | 硬件 | 接口 | 功能 |
| --- | --- | --- | --- |
| 主履带 | M3508 ×2 | CAN1 | 差速行驶 |
| 副履带 | M2006 ×2 | CAN1 | 独立速度控制 |
| 关节 | DM4310 ×2 | CAN2 | MIT 位置/阻尼控制 |
| Yaw | GM6020 ×1 | CAN1 | 位置-速度串级控制 |
| 抓取机构 | 舵机 ×3 | TIM1 PWM | Pitch、升降、夹爪 |
| 遥控 | DR16/兼容接收机 | USART3 + DMA | 人工输入 |
| 控制周期 | TIM6 | 2 ms 中断 | 整车任务调度 |

## C++ 文件结构

```text
Core/Src/main.cpp
User/Inc/{pid,remote,motor_dji,motor_dm,robot}.hpp
User/Src/{pid,remote,motor_dji,motor_dm,robot}.cpp
```

建议所有控制类位于 `namespace robot`，并由 `RobotController` 持有或引用其依赖。

## 整体执行逻辑

```text
上电
 ├─ HAL_Init()、时钟与 GPIO/DMA/CAN/TIM/UART 初始化
 ├─ RobotController::init()
 │   ├─ 初始化 PID、电机和遥控对象
 │   ├─ 启动 CAN 接收通知与遥控 DMA
 │   ├─ 使能 DM4310、启动三路 PWM
 │   └─ 写入舵机安全初值
 └─ HAL_TIM_Base_Start_IT(TIM6)
      │
      ▼ 每 2 ms
 RobotController::task()
 ├─ 更新遥控与电机在线状态
 ├─ 离线/不安全 → faultStop()
 ├─ updateStateMachine()
 ├─ 解算底盘、关节、Yaw、舵机指令
 └─ 发送 CAN/PWM 输出

CAN1 RX0 ─→ onCan1Rx() ─→ DJI 电机反馈
CAN2 RX0 ─→ onCan2Rx() ─→ DM4310 反馈
UART DMA ─→ onRxEvent() ─→ 遥控帧解析
```

## 状态机与遥控映射

| 输入 | 状态/行为 |
| --- | --- |
| 右拨杆 UP | `Standby`：履带停止、关节收折 |
| 右拨杆 MID | `Running`：右摇杆控制主履带差速 |
| 右拨杆 DOWN | `Climb`：主履带限速 |
| 左拨杆 UP / MID | `Flat` / `Ramp` 关节模式 |
| 左拨杆 DOWN | `Grab`：主履带停止，右摇杆控制 Pitch/升降 |
| 左摇杆水平 | 非待机时 Yaw 角度指令 |
| 左摇杆垂直 | 副履带速度 |
| 拨轮 | 抓取模式中开合夹爪 |

`Grab` 优先于右拨杆产生的行驶状态。

## 函数/方法参考

下表覆盖当前 C++ 控制模块的全部方法。

### `PidController`

| 方法 | 功能 |
| --- | --- |
| 构造函数 / `init(config)` | 保存参数，清零控制状态。 |
| `update(setpoint, measurement)` | 计算误差，执行位置式或增量式 PID，返回限幅输出。 |
| `reset()` | 清零误差历史、积分和输出，保留配置。 |
| `clamp(value, limit)` | 正负对称限幅。 |
| `calculatePosition(error)` | 计算位置式 P+I+D，限制积分和输出。 |
| `calculateIncrement(error)` | 以三次误差历史计算增量 PID。 |

### `RemoteReceiver`

| 方法 | 功能 |
| --- | --- |
| `init()` | 清空数据并启动 USART3 DMA 空闲接收。 |
| `parseFrame(bytes)` | 校验帧，解析五个通道、两个拨杆及收帧时刻。 |
| `updateOnlineStatus(nowMs)` | 按超时阈值更新在线状态。 |
| `data() const` | 返回最新遥控数据的只读引用。 |
| `onRxEvent(size)` | 处理 DMA/空闲事件并重新启动接收。 |
| `normalizeChannel(raw)` | 原始通道归一化到 `[-1, 1]`，应用死区。 |
| `extractChannel(bytes, index)` | 从位打包数据中读取 11 位通道。 |

### `DjiMotorBus`

| 方法 | 功能 |
| --- | --- |
| `initChassisMotors()` | 初始化两台 M3508 与速度 PID。 |
| `setChassisTargetSpeed(i,rpm)` | 设置一个主履带的目标速度。 |
| `sendChassisCurrents()` | 计算 PID，发送 `0x200` 电流帧。 |
| `onChassisFeedback(id,data)` | 解析 M3508 反馈，刷新在线心跳。 |
| `updateChassisOnlineStatus(now)` | 更新 M3508 在线状态。 |
| `chassisMotor(i) const` | 读取一台 M3508 状态。 |
| `initSubTrackMotors()` | 初始化两台 M2006 与速度 PID。 |
| `setSubTrackTargetSpeed(i,rpm)` | 设置副履带目标速度。 |
| `sendSubTrackCurrents()` | 计算 PID，发送 `0x1FF` 电流帧。 |
| `onSubTrackFeedback(id,data)` | 解析 M2006 反馈，刷新心跳。 |
| `updateSubTrackOnlineStatus(now)` | 更新 M2006 在线状态。 |
| `initYawMotor()` | 初始化 GM6020 位置外环/速度内环。 |
| `setYawTargetAngle(degree)` | 设置 Yaw 目标角度。 |
| `sendYawVoltage()` | 执行串级 PID，发送 `0x2FF` 电压帧。 |
| `onYawFeedback(id,data)` | 解析 GM6020 反馈并刷新心跳。 |
| `updateYawOnlineStatus(now)` | 更新 GM6020 在线状态。 |
| `yawMotor() const` | 读取 GM6020 状态。 |
| `parseFeedback(motor,data)` | 解码角度、速度、电流和温度。 |
| `sendFrame(id, outputs)` | 打包最多四路 16 位输出为 CAN 帧。 |
| `clampOutput(value,max)` | 限制电流或电压。 |
| `runSpeedPid(motor)` | 目标 RPM 转为控制输出。 |
| `runPositionSpeedCascade(motor)` | 位置环输出作为速度环设定值。 |

### `DmMotorBus`

| 方法 | 功能 |
| --- | --- |
| `init()` | 初始化两个 DM4310 的 ID、默认 Kp/Kd 与状态。 |
| `enable(joint)` | 发送 `0xFC` MIT 使能帧。 |
| `disable(joint)` | 发送 `0xFD` 失能帧。 |
| `clearFault(joint)` | 发送 `0xFB` 清故障帧。 |
| `setMitTarget(joint,pos,vel,torque,kp,kd)` | 设置 MIT 控制目标。 |
| `sendMitCommand(joint)` | 将 MIT 参数编码为 8 字节 CAN 帧。 |
| `onFeedback(id,data)` | 解析 ID、错误、位置、速度、力矩与心跳。 |
| `updateOnlineStatus(now)` | 依据反馈超时更新在线标志。 |
| `motor(joint) const` | 读取一个关节的状态。 |
| `floatToUint(value,min,max,bits)` | 将物理量映射为 MIT 无符号编码。 |
| `sendRawFrame(id,data)` | 使用 CAN2 发送原始帧。 |
| `sendSpecialCommand(id,cmd)` | 生成使能、失能或清故障特殊帧。 |

### `RobotController`

| 方法 | 功能 |
| --- | --- |
| `init()` | 初始化全部模块、CAN、PWM、舵机安全值并使能关节。 |
| `task()` | 2 ms 主控制：检查在线、状态机、指令解算和输出。 |
| `onCan1Rx(id,data)` | 分发 CAN1 的 M3508/M2006/GM6020 反馈。 |
| `onCan2Rx(id,data)` | 分发 CAN2 的 DM4310 反馈。 |
| `status() const` | 返回整车只读状态。 |
| `updateStateMachine()` | 根据拨杆更新车辆状态和关节模式。 |
| `resolveChassisCommand()` | 将右摇杆解算为左右履带 RPM。 |
| `resolveJointCommand()` | 生成关节目标角度与 Kp/Kd。 |
| `resolveYawCommand()` | 累加限幅后的 Yaw 目标。 |
| `resolveServoCommand()` | 解算 Pitch、升降、夹爪指令。 |
| `applyChassisMotors()` | 下发 M3508/M2006 电流命令。 |
| `applyJointMotors()` | 下发两台 DM4310 的 MIT 命令。 |
| `applyYawMotor()` | 下发 GM6020 Yaw 命令。 |
| `applyServos()` | 更新 TIM1 三路 PWM 比较值。 |
| `checkOnlineStatus()` | 汇总遥控及电机在线状态。 |
| `faultStop()` | 故障时置入安全输出。 |
| `setServoPulse(ch,us)` | 限幅后写入 PWM 脉宽。 |
| `applyDeadband(value,band)` | 死区内输入归零。 |
| `clampPulse(us,min,max)` | 限制舵机脉宽。 |
| `startCan()` | 配置过滤器、启动 CAN、打开 FIFO0 通知。 |

### `main.cpp` 与 HAL 桥接

| 函数 | 功能 |
| --- | --- |
| `main()` | 初始化 HAL/外设，调用 `RobotController::init()`，启动 TIM6 中断。 |
| `SystemClock_Config()` | 配置 F407 系统时钟。 |
| `HAL_CAN_RxFifo0MsgPendingCallback()` | 读取 CAN 帧并转交控制器。 |
| `HAL_UARTEx_RxEventCallback()` | 转交 USART3 DMA 接收事件。 |
| `HAL_TIM_PeriodElapsedCallback()` | TIM6 到期时执行 `task()`。 |
| `Error_Handler()` | HAL 初始化失败后的不可恢复安全停机。 |

## STM32 HAL 与 C++ 的桥接

CubeMX/HAL 以 C 接口调用回调。所有 HAL 回调必须保留 `extern "C"`，否则 C++ 名字改编会使 HAL 无法链接：

```cpp
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM6) {
        robotController.task();
    }
}
```

CAN 与 UART 回调同理。中断中只允许轻量处理；禁止阻塞日志、动态内存分配和长循环。

## CMake 要点

```cmake
project(ROBOT_FIRST LANGUAGES C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/main.cpp
    User/Src/pid.cpp User/Src/remote.cpp
    User/Src/motor_dji.cpp User/Src/motor_dm.cpp User/Src/robot.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE User/Inc)
```

嵌入式构建建议关闭异常与 RTTI，避免 `new/delete` 和隐式堆分配；使用静态对象、固定容量存储和轻量数据视图。

## 上车前检查

1. 确认遥控实际协议：18 字节 DBUS 与 25 字节 SBUS 不能混用，串口校验、停止位和反相也须匹配。
2. 将任何电机离线、CAN 发送失败和 DM 错误码纳入安全停机条件。
3. 故障停车必须向 GM6020 发送零电压或经验证的安全指令。
4. Yaw 需要上电零位对齐、跨零/多圈处理及机械和软件限位。
5. 在悬空、低速、低 Kp 下逐台确认转向与行程。

现场操作与排故步骤见 [DEBUG_README.md](DEBUG_README.md)。
