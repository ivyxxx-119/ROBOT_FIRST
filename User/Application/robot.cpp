#include "robot.hpp"

#include "main.h"
#include "imu_service.hpp"
#include "manipulator.hpp"
#include "debug.hpp"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    extern TIM_HandleTypeDef htim1;
    extern UART_HandleTypeDef huart3;
}

namespace Robot
{

void RobotController::init()
    {
        Imu_Init();
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

void RobotController::task()
    {
        ++debugTaskCount;

        /* =========================================================
         * 1. 更新传感器及在线状态
         * ========================================================= */
        Imu_Update();
        checkOnlineStatus();

        debugNowMs = HAL_GetTick();
        debugRemoteAgeMs =
            debugNowMs - remote_.data().lastUpdateMs;

        debugRemoteOnline = remoteOnline_ ? 1U : 0U;
        debugRemoteLastUpdateMs = remote_.data().lastUpdateMs;

        /* =========================================================
         * 2. 处理失联及状态切换
         * ========================================================= */
        if (!remoteOnline_)
        {
            faultStop();

            debugRobotState =
                static_cast<uint8_t>(state_);

            /* 避免调试界面显示上一次的辅助激活状态 */
            debugBmiYawRateDps = bmi_yaw_rate_dps;
            debugBmiStraightCorrectionRpm = 0.0f;
            debugBmiStraightActive = 0U;

            return;
        }

        debugSwitchRightRaw =
            static_cast<uint8_t>(remote_.data().switchRight);

        debugSwitchLeftRaw =
            static_cast<uint8_t>(remote_.data().switchLeft);

        debugRightV = remote_.data().rightV;

        RobotState oldState = state_;

        updateStateMachine();

        if (state_ != oldState)
        {
            onStateEnter(state_);
        }

        prevState_ = oldState;

        debugRobotState =
            static_cast<uint8_t>(state_);

        /* =========================================================
         * 3. 解算各机构目标
         * ========================================================= */
        resolveChassisCmd();
        resolveJointCmd();

        /* 升降与Yaw都需要使用仲裁后的左摇杆 */
        resolveLeftStickAxis();

        resolveYawCmd();
        resolveLiftCmd();
        resolveServoCmd();

        /* =========================================================
         * 4. 计算并发送输出
         * ========================================================= */
        applyChassisMotors();
        applyJointMotors();
        applyYawMotor();
        applyLiftMotor();
        applyServos();
    }

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
