/**
 * @file    robot.cpp
 * @brief   整车控制主模块实现
 *
 * 主要改动（相对旧版）：
 *  1. 新增 LiftMotorController（升降 M2006），CAN2，0x207
 *  2. 三台 M2006（副履带×2 + 升降×1）合并到同一个 0x1FF 帧发送
 *  3. 原 liftUs 舵机改为 foldUs 折叠舵机（TIM1_CH2）
 *  4. 状态枚举 Running→Drive，新增 Stair
 *  5. 副履带跟随主履带右摇杆，不再由 leftV 控制
 *  6. Stair：leftV 控制 DM 关节角度增减
 *  7. Grab：leftV 控制升降高度，leftH 控制 Yaw，dial 控制夹爪
 *  8. Yaw 仅 Grab 模式接受 leftH 输入
 *  9. Grab 进入/退出触发折叠舵机展开/收起（含安全高度互锁）
 * 10. resolveServoCmd / applyServos 通道调整
 */

#include "robot.hpp"
#include "main.h"
#include <cstring>
#include <cmath>

/* ======================== HAL 句柄 ======================== */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;

/* ======================== 舵机 TIM 通道 ======================== */
// TIM1_CH2 → 折叠舵机（原升降舵机通道复用）
// TIM1_CH3 → 夹爪舵机
static constexpr uint32_t kServoFoldCh = TIM_CHANNEL_2;
static constexpr uint32_t kServoGripperCh = TIM_CHANNEL_3;

/* ======================== 全局单例 ======================== */
static Robot::RobotController g_robot;

// 调试变量
volatile uint32_t debugCan2LastRxId = 0;
volatile uint32_t debugCan2RxCount = 0;
volatile int16_t debugOut205 = 0;
volatile int16_t debugOut206 = 0;
volatile int16_t debugOut207 = 0;
volatile uint32_t debugRx205Count = 0;
volatile uint32_t debugRx206Count = 0;
volatile int16_t debugSpeed205 = 0;
volatile uint16_t debugAngle205 = 0;

// 副履带详细调试
volatile float debugSubTargetRpm0 = 0.0f;
volatile float debugSubTargetRpm1 = 0.0f;
volatile int16_t debugSubFeedRpm0 = 0;
volatile int16_t debugSubFeedRpm1 = 0;
volatile int16_t debugSubOutput0 = 0;
volatile int16_t debugSubOutput1 = 0;

// 遥控器原始值调试（用于确认拨杆映射）
volatile uint8_t debugSwitchRightRaw = 0; // Byte5 bit[5:4] 原始值 1/2/3
volatile uint8_t debugSwitchLeftRaw = 0;  // Byte5 bit[7:6] 原始值 1/2/3
volatile uint8_t debugRobotState = 0;     // 当前状态机枚举值
volatile float debugRightV = 0.0f;        // 右摇杆垂直归一化值
volatile float debugLeftV = 0.0f;         // 左摇杆垂直归一化值（DM4310输入）
volatile float debugDmTargetDeg = 0.0f;   // DM4310当前目标角度

// 遥控器掉线调试
volatile uint32_t debugUartRxEventCount = 0U;
volatile uint16_t debugUartRxSize = 0U;
volatile uint32_t debugDbusAcceptedCount = 0U;
volatile uint32_t debugDbusRejectedSizeCount = 0U;
volatile uint8_t debugRemoteOnline = 0U;
volatile uint32_t debugRemoteLastUpdateMs = 0U;
volatile uint32_t debugNowMs = 0U;
volatile uint32_t debugRemoteAgeMs = 0U;
volatile uint32_t debugTaskCount = 0U;

// DM 关节调试（Left=0x001, Right=0x002）
volatile uint8_t debugDmLeftOnline = 0;      // Left 在线状态
volatile uint8_t debugDmRightOnline = 0;     // Right 在线状态
volatile uint8_t debugDmLeftError = 0;       // Left 错误码
volatile uint8_t debugDmRightError = 0;      // Right 错误码
volatile float debugDmLeftAngleDeg = 0.0f;   // Left 反馈角度(deg)
volatile float debugDmRightAngleDeg = 0.0f;  // Right 反馈角度(deg)
volatile float debugDmLeftVelDps = 0.0f;     // Left 反馈角速度(deg/s)
volatile float debugDmRightVelDps = 0.0f;    // Right 反馈角速度(deg/s)
volatile float debugDmLeftTorque = 0.0f;     // Left 反馈力矩(Nm)
volatile float debugDmRightTorque = 0.0f;    // Right 反馈力矩(Nm)
volatile float debugDmLeftTargetDeg = 0.0f;  // Left 发送目标角度(deg)
volatile float debugDmRightTargetDeg = 0.0f; // Right 发送目标角度(deg)
volatile uint32_t debugDmRxLeftCount = 0U;   // Left 反馈帧计数
volatile uint32_t debugDmRxRightCount = 0U;  // Right 反馈帧计数

// 主履带 M3508 调试（CAN1, 0x201/0x202）
volatile uint32_t debug201RxCount = 0U; // 0x201 反馈帧计数
volatile uint32_t debug202RxCount = 0U; // 0x202 反馈帧计数
volatile int16_t debug201SpeedRpm = 0;  // 0x201 反馈转速
volatile int16_t debug202SpeedRpm = 0;  // 0x202 反馈转速
volatile int16_t debugChTx201 = 0;      // 发给 0x201 的电流命令
volatile int16_t debugChTx202 = 0;      // 发给 0x202 的电流命令

/* ================================================================
 * C 接口
 * ================================================================ */
extern "C" void Robot_Init(void) { g_robot.init(); }
extern "C" void Robot_Task(void) { g_robot.task(); }

/* ================================================================
 * CAN / UART 回调
 * ================================================================ */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8] = {0U};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    if (hcan->Instance == hcan1.Instance)
        g_robot.can1RxDispatch(rxHeader.StdId, rxData);
    else if (hcan->Instance == hcan2.Instance)
        g_robot.can2RxDispatch(rxHeader.StdId, rxData, rxHeader.DLC);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8] = {0U};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK)
        return;

    // FIFO1：CAN2（M2006/DM4310）
    if (hcan->Instance == hcan2.Instance)
        g_robot.can2RxDispatch(rxHeader.StdId, rxData, rxHeader.DLC);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    g_robot.onUartRxEvent(huart, size);
}

/* ================================================================
 * 实现
 * ================================================================ */
namespace Robot
{

    /* ----------------------------------------------------------------
     * init
     * ---------------------------------------------------------------- */
    void RobotController::init()
    {
        hcan1_ = &hcan1;
        hcan2_ = &hcan2;
        htim1_ = &htim1;
        huart3_ = &huart3;

        state_ = RobotState::Init;
        prevState_ = RobotState::Init;
        gripperState_ = GripperState::Open;
        foldState_ = FoldState::Folded;
        targetYawDeg_ = 0.0f;
        dmTargetDeg_ = kJointFoldDeg;

        servoCmd_.foldUs = kFoldFlatUs;
        servoCmd_.gripperUs = kGripperOpenUs;

        // 初始化电机控制器
        chassis_.init(hcan1_);
        subTrack_.init(hcan2_);
        lift_.init(hcan2_);
        yaw_.init(hcan1_);
        joint_.init(hcan2_);
        remote_.init(huart3_);

        startCan();

        // DM 电机内部 MCU 上电启动需要时间，必须等其 CAN 栈就绪后再发使能帧。
        // 调试器连接时复位延迟约 500 ms 掩盖了此问题，冷上电时电机会静默丢弃帧。
        HAL_Delay(1000U);

        // 使能 DM 关节电机
        joint_.clearFault(DmIndex::Left);
        joint_.clearFault(DmIndex::Right);
        HAL_Delay(20U);
        joint_.enable(DmIndex::Left);
        joint_.enable(DmIndex::Right);

        // 启动舵机 PWM（只用 CH2 和 CH3）
        HAL_TIM_PWM_Start(htim1_, kServoFoldCh);
        HAL_TIM_PWM_Start(htim1_, kServoGripperCh);

        setServoPulse(kServoFoldCh, kFoldFlatUs);
        setServoPulse(kServoGripperCh, kGripperOpenUs);

        state_ = RobotState::Standby;
        prevState_ = RobotState::Standby;
    }

    /* ----------------------------------------------------------------
     * task（每 2 ms 调用）
     * ---------------------------------------------------------------- */
    void RobotController::task()
    {
        ++debugTaskCount;

        checkOnlineStatus();

        debugNowMs = HAL_GetTick();
        debugRemoteAgeMs =
            debugNowMs - remote_.data().lastUpdateMs;
        debugRemoteOnline = remoteOnline_ ? 1U : 0U;
        debugRemoteLastUpdateMs = remote_.data().lastUpdateMs;

        if (!remoteOnline_)
        {
            faultStop();
            return;
        }

        debugRemoteOnline = remoteOnline_ ? 1U : 0U;
        debugRemoteLastUpdateMs = remote_.data().lastUpdateMs;

        // 等两台 DM 都在线后同步目标到实际位置，之后才允许发指令
        // if (firstTask_)
        // {
        //     const DmMotorInstance *mL = joint_.getInstance(DmIndex::Left);
        //     const DmMotorInstance *mR = joint_.getInstance(DmIndex::Right);
        //     bool lOnline = (mL != nullptr && mL->isOnline());
        //     bool rOnline = (mR != nullptr && mR->isOnline());
        //     if (lOnline && rOnline)
        //     {
        //         dmTargetDeg_ = mL->feedback().angleDeg;
        //         firstTask_ = false;
        //     }
        //     // 未就绪前跳过本周期所有输出，避免以 0 度目标驱动电机
        //     return;
        // }

        if (!remoteOnline_)
        {
            faultStop();
            return;
        }

        // 刷新遥控器原始调试值
        debugSwitchRightRaw = static_cast<uint8_t>(remote_.data().switchRight);
        debugSwitchLeftRaw = static_cast<uint8_t>(remote_.data().switchLeft);
        debugRightV = remote_.data().rightV;
        debugRobotState = static_cast<uint8_t>(state_);

        RobotState oldState = state_;
        updateStateMachine();

        // 检测状态切换，执行进入动作
        if (state_ != oldState)
        {
            onStateEnter(state_);
        }
        prevState_ = oldState;

        resolveChassisCmd();
        resolveJointCmd();
        resolveLeftStickAxis();
        resolveYawCmd();
        resolveLiftCmd();
        resolveServoCmd();

        applyChassisMotors();
        applyJointMotors();
        applyYawMotor();
        applyLiftMotor();
        applyServos();
    }

    /* ----------------------------------------------------------------
     * 状态进入回调
     * ---------------------------------------------------------------- */
    void RobotController::onStateEnter(RobotState newState)
    {
        switch (newState)
        {
        case RobotState::Stair:
            // 进入爬楼：从当前反馈角度接管 DM 目标，避免突变
            {
                const DmMotorInstance *mL = joint_.getInstance(DmIndex::Left);
                if (mL != nullptr && mL->isOnline())
                    dmTargetDeg_ = mL->feedback().angleDeg;
                else
                    dmTargetDeg_ = kJointFoldDeg;
            }
            break;

        case RobotState::Grab:
            // 进入抓取：同步升降目标到当前位置，折叠舵机展开
            lift_.syncTargetToCurrent();
            // 仅当升降在安全高度时才展开；否则程序仍展开但应加机械互锁
            servoCmd_.foldUs = kFoldDeployUs;
            foldState_ = FoldState::Deployed;
            // 同步 Yaw 目标，防止模式切换时跳变
            if (yaw_.isOnline())
            {
                targetYawDeg_ = yaw_.currentAngleDeg();
                yaw_.syncTargetToCurrent();
            }
            break;

        case RobotState::Standby:
        case RobotState::Drive:
            // 离开 Grab → 收起折叠舵机
            // 注意：理想情况下应先等升降回安全高度；
            // 此处简化为直接发收起指令，建议加到位开关后改为状态机等待。
            if (foldState_ == FoldState::Deployed)
            {
                servoCmd_.foldUs = kFoldFlatUs;
                foldState_ = FoldState::Folded;
            }
            // DM 回到收起位
            dmTargetDeg_ = kJointFoldDeg;
            // Yaw 不再接受新增量，锁定当前位置
            if (yaw_.isOnline())
            {
                targetYawDeg_ = yaw_.currentAngleDeg();
                yaw_.syncTargetToCurrent();
            }
            break;

        default:
            break;
        }
    }

    /* ----------------------------------------------------------------
     * CAN 接收分发
     * ---------------------------------------------------------------- */
    void RobotController::can1RxDispatch(uint32_t canId, const uint8_t *pData)
    {
        if (canId == 0x201U)
        {
            debug201RxCount++;
            debug201SpeedRpm = static_cast<int16_t>(
                (static_cast<uint16_t>(pData[2]) << 8U) | pData[3]);
        }
        else if (canId == 0x202U)
        {
            debug202RxCount++;
            debug202SpeedRpm = static_cast<int16_t>(
                (static_cast<uint16_t>(pData[2]) << 8U) | pData[3]);
        }

        if (canId >= kChassisFeedbackBase &&
            canId < kChassisFeedbackBase + kChassisMotorCount)
        {
            chassis_.updateFeedback(canId, pData);
            return;
        }
        if (canId == kYawFeedbackId)
        {
            yaw_.updateFeedback(canId, pData);
            return;
        }
    }

    void RobotController::can2RxDispatch(uint32_t canId,
                                         const uint8_t *pData,
                                         uint8_t dlc)
    {
        // 调试变量
        debugCan2LastRxId = canId;
        debugCan2RxCount++;

        if (canId == 0x205U)
        {
            ++debugRx205Count;
            debugAngle205 = static_cast<uint16_t>(
                (static_cast<uint16_t>(pData[0]) << 8U) |
                static_cast<uint16_t>(pData[1]));
            debugSpeed205 = static_cast<int16_t>(
                (static_cast<uint16_t>(pData[2]) << 8U) |
                static_cast<uint16_t>(pData[3]));
        }
        else if (canId == 0x206U)
        {
            ++debugRx206Count;
        }

        // 副履带 M2006：0x205~0x206
        if (canId >= kSubTrackFeedbackBase &&
            canId < kSubTrackFeedbackBase + kSubTrackMotorCount)
        {
            subTrack_.updateFeedback(canId, pData);
            return;
        }

        // 升降 M2006：0x207
        if (canId == kLiftFeedbackId)
        {
            lift_.updateFeedback(canId, pData);
            return;
        }

        // DM4310 关节
        if (canId == kDmJointLeftRxCanId)
            ++debugDmRxLeftCount;
        else if (canId == kDmJointRightRxCanId)
            ++debugDmRxRightCount;
        joint_.updateFeedback(canId, pData, dlc);
    }

    /* ----------------------------------------------------------------
     * UART 回调
     * ---------------------------------------------------------------- */
    void RobotController::onUartRxEvent(
        UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart == nullptr || huart->Instance != huart3_->Instance)
            return;

        ++debugUartRxEventCount;
        debugUartRxSize = size;

        if (size == kDbusFrameLen)
        {
            const uint32_t previousMs = remote_.data().lastUpdateMs;
            remote_.parseDbus(remote_.rxBuf(), size);

            // 初步判断是否通过解析；更精确的成功计数建议放进 parseDbus()
            if (remote_.data().online &&
                remote_.data().lastUpdateMs != previousMs)
            {
                ++debugDbusAcceptedCount;
            }
        }
        else
        {
            ++debugDbusRejectedSizeCount;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(
            huart3_, remote_.rxBuf(), remote_.rxBufSize());
        __HAL_DMA_DISABLE_IT(huart3_->hdmarx, DMA_IT_HT);
    }

    /* ----------------------------------------------------------------
     * 状态机
     * 右拨杆 Up → 永远 Standby，同时清除 Grab 请求
     * 右拨杆 Mid → Drive；Down → Stair
     * 左拨杆从非 Down 拨到 Down（上升沿）才请求 Grab；
     * 右拨杆回到 Up 后必须重新拨左拨杆才能再次进入 Grab
     * ---------------------------------------------------------------- */
    void RobotController::updateStateMachine()
    {
        const RemoteData &rc = remote_.data();

        // 右拨杆 Up：Standby，清除 Grab 请求
        if (rc.switchRight == SwitchPos::Up)
        {
            state_ = RobotState::Standby;
            grabRequested_ = false;
            prevSwitchLeft_ = rc.switchLeft;
            return;
        }

        // 左拨杆边沿检测：从非 Down → Down 才置请求
        if (rc.switchLeft == SwitchPos::Down &&
            prevSwitchLeft_ != SwitchPos::Down)
        {
            grabRequested_ = true;
        }
        // 左拨杆离开 Down 时清除请求
        if (rc.switchLeft != SwitchPos::Down)
        {
            grabRequested_ = false;
        }
        prevSwitchLeft_ = rc.switchLeft;

        // 按右拨杆设置基础状态
        switch (rc.switchRight)
        {
        case SwitchPos::Mid:
            state_ = RobotState::Drive;
            break;
        case SwitchPos::Down:
            state_ = RobotState::Stair;
            break;
        default:
            state_ = RobotState::Standby;
            break;
        }

        // Grab 请求覆盖（不覆盖 Standby）
        if (grabRequested_)
        {
            state_ = RobotState::Grab;
        }
    }

    /* ----------------------------------------------------------------
     * Grab 模式左摇杆轴仲裁
     * 对两轴分别过死区后，绝对值大的轴为主导轴，另一轴清零。
     * 结果写入 grabLeftVArb_（升降）和 grabLeftHArb_（Yaw）。
     * 非 Grab 模式直接清零。
     * ---------------------------------------------------------------- */
    void RobotController::resolveLeftStickAxis()
    {
        if (state_ != RobotState::Grab)
        {
            grabLeftVArb_ = 0.0f;
            grabLeftHArb_ = 0.0f;
            return;
        }

        const RemoteData &rc = remote_.data();
        float v = applyDeadband(rc.leftV, kJoystickDeadband);
        float h = applyDeadband(rc.leftH, kJoystickDeadband);

        // 绝对值大的轴主导，另一轴清零
        if (fabsf(v) >= fabsf(h))
        {
            grabLeftVArb_ = v;
            grabLeftHArb_ = 0.0f;
        }
        else
        {
            grabLeftVArb_ = 0.0f;
            grabLeftHArb_ = h;
        }
    }

    /* ----------------------------------------------------------------
     * 底盘指令解算
     * Drive/Stair/Grab 均由右摇杆控制，速度限制不同
     * ---------------------------------------------------------------- */
    void RobotController::resolveChassisCmd()
    {
        if (state_ == RobotState::Standby)
        {
            chassisCmd_.leftRpm = 0.0f;
            chassisCmd_.rightRpm = 0.0f;
            return;
        }

        const RemoteData &rc = remote_.data();

        float fwd = applyDeadband(rc.rightV, kJoystickDeadband);
        float turn = applyDeadband(rc.rightH, kJoystickDeadband);

        float maxRpm;
        switch (state_)
        {
        case RobotState::Grab:
            maxRpm = kTrackGrabRpm;
            break;
        case RobotState::Stair:
            maxRpm = kTrackClimbRpm;
            break;
        default: // Drive
            maxRpm = kTrackMaxRpm;
            break;
        }

        float fwdRpm = fwd * maxRpm;
        float turnRpm = turn * maxRpm * kTurnScale;

        chassisCmd_.leftRpm = fwdRpm + turnRpm;
        chassisCmd_.rightRpm = fwdRpm - turnRpm;
    }

    /* ----------------------------------------------------------------
     * DM 关节指令解算
     * 仅 Stair 模式接受 leftV 输入；其余模式回收起位
     * ---------------------------------------------------------------- */
    void RobotController::resolveJointCmd()
    {
        if (state_ == RobotState::Stair)
        {
            const RemoteData &rc = remote_.data();
            float input = applyDeadband(rc.leftV, kJoystickDeadband);
            debugLeftV = rc.leftV;

            // 按时间累积目标角度
            dmTargetDeg_ += input * kJointRateDegPerSec * kTaskPeriodS;
            dmTargetDeg_ = clampF(dmTargetDeg_, kJointMinDeg, kJointMaxDeg);
            debugDmTargetDeg = dmTargetDeg_;

            jointCmd_.leftDeg = dmTargetDeg_;
            jointCmd_.rightDeg = dmTargetDeg_;
            jointCmd_.velDegPerS = input * kJointRateDegPerSec * 0.15f;
            jointCmd_.kp = kJointKp;
            jointCmd_.kd = kJointKd;
        }
        else if (state_ == RobotState::Standby)
        {
            // Standby：目标锁定在进入时冻结的 dmTargetDeg_，kp>0 提供静态保持力抵抗重力
            jointCmd_.leftDeg = dmTargetDeg_;
            jointCmd_.rightDeg = dmTargetDeg_;
            jointCmd_.kp = kJointFoldKp;
            jointCmd_.kd = kJointFoldKd;
        }
        else
        {
            // Drive / Grab：跟踪实际位置，目标误差为零，小 kp 提供保持力
            const DmMotorInstance *mL = joint_.getInstance(DmIndex::Left);
            const DmMotorInstance *mR = joint_.getInstance(DmIndex::Right);
            if (mL != nullptr && mL->isOnline())
                dmTargetDeg_ = mL->feedback().angleDeg;
            jointCmd_.leftDeg = (mL != nullptr && mL->isOnline()) ? mL->feedback().angleDeg : dmTargetDeg_;
            jointCmd_.rightDeg = (mR != nullptr && mR->isOnline()) ? mR->feedback().angleDeg : dmTargetDeg_;
            jointCmd_.kp = kJointFoldKp;
            jointCmd_.kd = kJointFoldKd;
        }
    }

    /* ----------------------------------------------------------------
     * Yaw 指令解算
     * 仅 Grab 模式接受 leftH 输入
     * ---------------------------------------------------------------- */
    void RobotController::resolveYawCmd()
    {
        if (state_ != RobotState::Grab)
            return;
        if (!yaw_.isOnline())
            return;

        float yawInput = grabLeftHArb_;

        targetYawDeg_ += yawInput * kYawStepDeg;
        targetYawDeg_ = clampF(targetYawDeg_, -kYawMaxDeg, kYawMaxDeg);
    }

    /* ----------------------------------------------------------------
     * 升降指令解算
     * 仅 Grab 模式由 leftV 控制目标高度；其余模式锁定当前高度
     * ---------------------------------------------------------------- */
    void RobotController::resolveLiftCmd()
    {
        if (state_ == RobotState::Grab)
        {
            lift_.updateTargetByJoystick(grabLeftVArb_, kTaskPeriodS);
        }
        // 其余模式不调用 updateTargetByJoystick，目标保持不变
    }

    /* ----------------------------------------------------------------
     * 舵机指令解算
     * foldUs 由状态机 onStateEnter 控制；此处只处理夹爪
     * ---------------------------------------------------------------- */
    void RobotController::resolveServoCmd()
    {
        const RemoteData &rc = remote_.data();

        if (state_ == RobotState::Grab)
        {
            // 拨轮阈值切换夹爪
            if (rc.dial > kDialGripperThresh)
            {
                gripperState_ = GripperState::Closed;
                servoCmd_.gripperUs = kGripperCloseUs;
            }
            else if (rc.dial < -kDialGripperThresh)
            {
                gripperState_ = GripperState::Open;
                servoCmd_.gripperUs = kGripperOpenUs;
            }
        }
        // 非 Grab 模式夹爪保持上次状态，不强制复位
    }

    /* ----------------------------------------------------------------
     * 执行输出：底盘
     * 三台 M2006（副履带×2 + 升降×1）合并到同一个 0x1FF 帧
     * ---------------------------------------------------------------- */
    void RobotController::applyChassisMotors()
    {
        // M3508 主履带
        chassis_.setTargetSpeedRpm(0U, chassisCmd_.leftRpm);
        chassis_.setTargetSpeedRpm(1U, -chassisCmd_.rightRpm); // 右侧取反修正安装方向
        chassis_.sendAllCurrent();

        // 副履带目标转速 = 主履带指令 × 比例系数（同一摇杆输入）
        float subLeft = chassisCmd_.leftRpm * kSubTrackRatio;
        float subRight = chassisCmd_.rightRpm * kSubTrackRatio;

        // Standby 时副履带也停止，清零积分器防止残留电流
        if (state_ == RobotState::Standby)
        {
            subLeft = 0.0f;
            subRight = 0.0f;
            subTrack_.setTargetSpeedRpm(0U, 0.0f);
            subTrack_.setTargetSpeedRpm(1U, 0.0f);
            subTrack_.resetPid();
            int16_t zeros[4] = {0, 0, lift_.getTargetOutput(), 0};
            debugOut205 = 0;
            debugOut206 = 0;
            lift_.calcOutput();
            zeros[2] = lift_.getTargetOutput();
            DjiSendCanFrame(hcan2_, kSubTrackCmdId, zeros, 4U);
            return;
        }

        subTrack_.setTargetSpeedRpm(0U, subLeft);
        subTrack_.setTargetSpeedRpm(1U, -subRight); // 右侧取反

        // 计算各自 PID 输出（不独立发送）
        subTrack_.calcOutput();
        lift_.calcOutput();

        // 副履带详细调试
        {
            const DjiMotorInstance *m0 = subTrack_.getInstance(0U);
            const DjiMotorInstance *m1 = subTrack_.getInstance(1U);
            if (m0 != nullptr)
            {
                debugSubTargetRpm0 = m0->targetSpeedRpm;
                debugSubFeedRpm0 = m0->feedback().speedRpm;
                debugSubOutput0 = m0->targetOutput;
            }
            if (m1 != nullptr)
            {
                debugSubTargetRpm1 = m1->targetSpeedRpm;
                debugSubFeedRpm1 = m1->feedback().speedRpm;
                debugSubOutput1 = m1->targetOutput;
            }
        }

        // 合并三台 M2006 到同一个 0x1FF 帧：slot0=副左, slot1=副右, slot2=升降, slot3=0
        int16_t outputs[4] = {
            subTrack_.getTargetOutput(0U),
            subTrack_.getTargetOutput(1U),
            lift_.getTargetOutput(),
            0};

        // 调试：记录准备发给三台 M2006 的电流指令
        debugOut205 = outputs[0]; // 0x205：左副履带
        debugOut206 = outputs[1]; // 0x206：右副履带
        debugOut207 = outputs[2]; // 0x207：升降

        DjiSendCanFrame(hcan2_, kSubTrackCmdId, outputs, 4U);
    }

    /* ----------------------------------------------------------------
     * 执行输出：DM 关节
     * ---------------------------------------------------------------- */
    void RobotController::applyJointMotors()
    {
        // 刷新 DM 调试变量
        const DmMotorInstance *mL = joint_.getInstance(DmIndex::Left);
        const DmMotorInstance *mR = joint_.getInstance(DmIndex::Right);
        if (mL != nullptr)
        {
            debugDmLeftOnline = mL->isOnline() ? 1U : 0U;
            debugDmLeftError = mL->feedback().errorCode;
            debugDmLeftAngleDeg = mL->feedback().angleDeg;
            debugDmLeftVelDps = mL->feedback().velocityDegPerS;
            debugDmLeftTorque = mL->feedback().torque;
            debugDmLeftTargetDeg = jointCmd_.leftDeg;
        }
        if (mR != nullptr)
        {
            debugDmRightOnline = mR->isOnline() ? 1U : 0U;
            debugDmRightError = mR->feedback().errorCode;
            debugDmRightAngleDeg = mR->feedback().angleDeg;
            debugDmRightVelDps = mR->feedback().velocityDegPerS;
            debugDmRightTorque = mR->feedback().torque;
            debugDmRightTargetDeg = jointCmd_.rightDeg;
        }

        joint_.setMitTarget(DmIndex::Left,
                            jointCmd_.leftDeg,
                            jointCmd_.velDegPerS, 0.0f,
                            jointCmd_.kp, jointCmd_.kd);

        joint_.setMitTarget(DmIndex::Right,
                            jointCmd_.rightDeg,
                            jointCmd_.velDegPerS, 0.0f,
                            jointCmd_.kp, jointCmd_.kd);

        joint_.sendMitCommand(DmIndex::Left);
        joint_.sendMitCommand(DmIndex::Right);
    }

    /* ----------------------------------------------------------------
     * 执行输出：Yaw
     * ---------------------------------------------------------------- */
    void RobotController::applyYawMotor()
    {
        if (state_ == RobotState::Standby)
        {
            yaw_.syncTargetToCurrent();
            targetYawDeg_ = yaw_.currentAngleDeg();
            yaw_.sendZeroCurrent();
            return;
        }

        if (!yaw_.isOnline())
        {
            yaw_.sendZeroCurrent();
            return;
        }

        // 仅 Grab 模式才更新目标并运行闭环；其余模式锁定当前位置
        if (state_ == RobotState::Grab)
        {
            yaw_.setTargetAngleDeg(targetYawDeg_);
            yaw_.sendCurrent();
        }
        else
        {
            // Drive / Stair：保持目标=当前角度，输出零或极小电流
            yaw_.syncTargetToCurrent();
            targetYawDeg_ = yaw_.currentAngleDeg();
            yaw_.sendZeroCurrent();
        }
    }

    /* ----------------------------------------------------------------
     * 执行输出：升降电机
     * 实际发送已在 applyChassisMotors() 内合并，此处为占位
     * calcOutput() 已在 applyChassisMotors() 中调用
     * ---------------------------------------------------------------- */
    void RobotController::applyLiftMotor()
    {
        // 升降电机的 CAN 帧已在 applyChassisMotors() 内与副履带合并发送
        // 此函数保留供后续扩展（如独立发送、状态检查等）
    }

    /* ----------------------------------------------------------------
     * 执行输出：舵机
     * ---------------------------------------------------------------- */
    void RobotController::applyServos()
    {
        setServoPulse(kServoFoldCh, servoCmd_.foldUs);
        setServoPulse(kServoGripperCh, servoCmd_.gripperUs);
    }

    /* ----------------------------------------------------------------
     * 在线状态检查
     * ---------------------------------------------------------------- */
    void RobotController::checkOnlineStatus()
    {
        remote_.updateOnlineStatus();
        chassis_.updateOnlineStatus();
        subTrack_.updateOnlineStatus();
        lift_.updateOnlineStatus();
        yaw_.updateOnlineStatus();
        joint_.updateOnlineStatus();

        remoteOnline_ = remote_.data().online;
        motorAllOnline_ = chassis_.isAllOnline();
    }

    /* ----------------------------------------------------------------
     * 故障停车
     * ---------------------------------------------------------------- */
    void RobotController::faultStop()
    {
        // 主履带清零
        chassis_.setTargetSpeedRpm(0U, 0.0f);
        chassis_.setTargetSpeedRpm(1U, 0.0f);
        chassis_.sendAllCurrent();

        // 副履带 + 升降合并清零
        subTrack_.setTargetSpeedRpm(0U, 0.0f);
        subTrack_.setTargetSpeedRpm(1U, 0.0f);
        subTrack_.calcOutput();
        // 升降：保持当前目标位置（不更改 targetPosCnt_），输出由位置环产生
        // 若需要失联时强制停止升降，可调用 lift_.syncTargetToCurrent()
        lift_.calcOutput();

        int16_t outputs[4] = {
            subTrack_.getTargetOutput(0U),
            subTrack_.getTargetOutput(1U),
            lift_.getTargetOutput(),
            0};
        DjiSendCanFrame(hcan2_, kSubTrackCmdId, outputs, 4U);

        // Yaw 锁定
        yaw_.syncTargetToCurrent();
        targetYawDeg_ = yaw_.currentAngleDeg();
        yaw_.sendZeroCurrent();

        // DM 关节：纯阻尼保持当前位置
        for (uint8_t i = 0U; i < kDmMotorCount; i++)
        {
            DmIndex idx = static_cast<DmIndex>(i);
            const DmMotorInstance *m = joint_.getInstance(idx);
            float holdDeg = (m != nullptr) ? m->feedback().angleDeg : 0.0f;

            joint_.setMitTarget(idx, holdDeg, 0.0f, 0.0f, 0.0f, 2.0f);
            joint_.sendMitCommand(idx);
        }

        // 折叠舵机：失联时不改变当前状态，避免盲目收起/展开
        // servoCmd_.foldUs 保持不变

        state_ = RobotState::Standby;
    }

    /* ----------------------------------------------------------------
     * CAN 过滤器 & 启动
     * ---------------------------------------------------------------- */
    void RobotController::startCan()
    {
        CAN_FilterTypeDef f{};
        f.FilterMode = CAN_FILTERMODE_IDMASK;
        f.FilterScale = CAN_FILTERSCALE_32BIT;
        f.FilterIdHigh = 0x0000U;
        f.FilterIdLow = 0x0000U;
        f.FilterMaskIdHigh = 0x0000U;
        f.FilterMaskIdLow = 0x0000U;
        f.FilterActivation = ENABLE;
        f.SlaveStartFilterBank = 14U;

        // CAN1/CAN2 同一物理总线 → 统一用 FIFO0
        f.FilterBank = 0U;
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        HAL_CAN_ConfigFilter(hcan1_, &f);
        HAL_CAN_Start(hcan1_);
        HAL_CAN_ActivateNotification(hcan1_, CAN_IT_RX_FIFO0_MSG_PENDING);

        f.FilterBank = 14U;
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        HAL_CAN_ConfigFilter(hcan2_, &f);
        HAL_CAN_Start(hcan2_);
        HAL_CAN_ActivateNotification(hcan2_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    /* ----------------------------------------------------------------
     * 工具函数
     * ---------------------------------------------------------------- */
    void RobotController::setServoPulse(uint32_t channel, uint32_t pulseUs)
    {
        pulseUs = clampPulse(pulseUs, kServoPwmMinUs, kServoPwmMaxUs);
        __HAL_TIM_SET_COMPARE(htim1_, channel, pulseUs);
    }

    uint32_t RobotController::clampPulse(uint32_t v, uint32_t minV, uint32_t maxV)
    {
        if (v < minV)
            return minV;
        if (v > maxV)
            return maxV;
        return v;
    }

    float RobotController::applyDeadband(float v, float db)
    {
        if (v > -db && v < db)
            return 0.0f;
        return v;
    }

    float RobotController::clampF(float v, float lo, float hi)
    {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    }

} // namespace Robot