#include "chassis.hpp"

#include "imu_service.hpp"
#include "debug.hpp"

#include <cmath>

namespace
{

/* 保持原有参数，直行辅助默认关闭。 */
static constexpr bool kBmiStraightAssistEnabled = false;
static constexpr float kBmiStraightKp = 2.0f;
static constexpr float kBmiStraightLimitRatio = 0.05f;
static constexpr float kBmiStraightCorrectionSign = 1.0f;

} // namespace

namespace Robot
{

void RobotController::resolveChassisCmd()
    {
        debugBmiYawRateDps = bmi_yaw_rate_dps;
        debugBmiStraightCorrectionRpm = 0.0f;
        debugBmiStraightActive = 0U;

        if (state_ == RobotState::Standby)
        {
            chassisCmd_.leftRpm = 0.0f;
            chassisCmd_.rightRpm = 0.0f;
            return;
        }

        const RemoteData &rc = remote_.data();

        float fwd = applyDeadband(
            rc.rightV, kJoystickDeadband);

        float turn = applyDeadband(
            rc.rightH, kJoystickDeadband);

        float maxRpm;

        switch (state_)
        {
        case RobotState::Grab:
            maxRpm = kTrackGrabRpm;
            break;

        case RobotState::Stair:
            maxRpm = kTrackClimbRpm;
            break;

        default:
            maxRpm = kTrackMaxRpm;
            break;
        }

        float fwdRpm = fwd * maxRpm;
        float turnRpm = turn * maxRpm * kTurnScale;

        /*
         * 第一阶段只在Drive直行时启用。
         *
         * 不在Stair启用：
         * 台阶上的滑移和姿态变化更复杂。
         *
         * 不在Grab启用：
         * 先避免影响操作手精细对位。
         *
         * 松开前进摇杆或主动打转向时，不产生修正。
         */
        if (kBmiStraightAssistEnabled &&
            state_ == RobotState::Drive &&
            turn == 0.0f &&
            fabsf(fwd) > 0.10f &&
            Imu_IsUsable())
        {
            float limit =
                maxRpm * kBmiStraightLimitRatio;

            float correction =
                kBmiStraightCorrectionSign *
                kBmiStraightKp *
                bmi_yaw_rate_dps;

            correction = clampF(
                correction, -limit, limit);

            turnRpm += correction;

            debugBmiStraightCorrectionRpm = correction;
            debugBmiStraightActive = 1U;
        }

        float leftRpm = fwdRpm + turnRpm;
        float rightRpm = fwdRpm - turnRpm;

        /*
         * 原来的fwd + turn可能超过maxRpm。
         * 等比例缩放，保持左右速度比例。
         */
        float peak = fmaxf(
            fabsf(leftRpm), fabsf(rightRpm));

        if (peak > maxRpm && peak > 0.0f)
        {
            float scale = maxRpm / peak;

            leftRpm *= scale;
            rightRpm *= scale;
        }

        chassisCmd_.leftRpm = leftRpm;
        chassisCmd_.rightRpm = rightRpm;
    }

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

} // namespace Robot
