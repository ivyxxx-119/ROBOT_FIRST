#include "chassis.hpp"

#include "imu_service.hpp"
#include "debug.hpp"

#include <cmath>

namespace
{
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
        default:
            maxRpm = kTrackMaxRpm;
            break;
        }

        float fwdRpm = fwd * maxRpm;
        float turnRpm = turn * maxRpm * kTurnScale;

        if (kBmiStraightAssistEnabled &&
            state_ == RobotState::Drive &&
            turn == 0.0f &&
            fabsf(fwd) > 0.10f &&
            Imu_IsUsable())
        {
            float limit = maxRpm * kBmiStraightLimitRatio;

            float correction =
                kBmiStraightCorrectionSign *
                kBmiStraightKp *
                bmi_yaw_rate_dps;

            correction = clampF(correction, -limit, limit);
            turnRpm += correction;

            debugBmiStraightCorrectionRpm = correction;
            debugBmiStraightActive = 1U;
        }

        float leftRpm = fwdRpm + turnRpm;
        float rightRpm = fwdRpm - turnRpm;

        float peak = fmaxf(fabsf(leftRpm), fabsf(rightRpm));
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
        /* ---- 主履带 M3508 ---- */
        chassis_.setTargetSpeedRpm(0U, chassisCmd_.leftRpm);
        chassis_.setTargetSpeedRpm(1U, -chassisCmd_.rightRpm);
        chassis_.calcOutput();

        /* ---- 副履带 M2006 ---- */
        if (state_ == RobotState::Standby)
        {
            subTrack_.clearOutput();
        }
        else
        {
            subTrack_.setTargetSpeedRpm(
                0U, chassisCmd_.leftRpm * kSubTrackRatio);
            subTrack_.setTargetSpeedRpm(
                1U, -chassisCmd_.rightRpm * kSubTrackRatio);
            subTrack_.calcOutput();
        }

        /* ---- 调试变量 ---- */
        debugSubTargetRpm0 = subTrack_.getInstance(0U)
                                 ? subTrack_.getInstance(0U)->targetSpeedRpm
                                 : 0.0f;
        debugSubTargetRpm1 = subTrack_.getInstance(1U)
                                 ? subTrack_.getInstance(1U)->targetSpeedRpm
                                 : 0.0f;
        debugSubFeedRpm0 = subTrack_.getInstance(0U)
                               ? subTrack_.getInstance(0U)->feedback().speedRpm
                               : 0;
        debugSubFeedRpm1 = subTrack_.getInstance(1U)
                               ? subTrack_.getInstance(1U)->feedback().speedRpm
                               : 0;
        debugSubOutput0 = subTrack_.getTargetOutput(0U);
        debugSubOutput1 = subTrack_.getTargetOutput(1U);
    }

} // namespace Robot