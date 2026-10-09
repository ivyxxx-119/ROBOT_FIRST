/**
 * @file    manipulator.cpp
 * @brief   机械臂相关函数实现
 *          包含：关节/Yaw/升降/舵机的解算与输出计算
 *          通信函数（CAN/UART）均在 communication.cpp
 */
#include "manipulator.hpp"
#include "robot.hpp"
#include "main.h"
#include "debug.hpp"

#include <cmath>

namespace Robot
{

    /* ================================================================
     * resolveLeftStickAxis
     * Grab 模式下对左摇杆 V/H 轴做单轴仲裁：
     * 绝对值较大的轴独占，另一轴清零，防止升降和 Yaw 同时运动。
     * ================================================================ */
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

    /* ================================================================
     * resolveJointCmd
     * ================================================================ */
    void RobotController::resolveJointCmd()
    {
        if (state_ == RobotState::Stair)
        {
            const RemoteData &rc = remote_.data();
            float leftV = applyDeadband(rc.leftV, kJoystickDeadband);

            dmTargetDeg_ += leftV * kJointRateDegPerSec * kTaskPeriodS;
            dmTargetDeg_ = clampF(dmTargetDeg_, kJointMinDeg, kJointMaxDeg);

            jointCmd_.leftDeg = dmTargetDeg_;
            jointCmd_.rightDeg = dmTargetDeg_;
            jointCmd_.velDegPerS = 0.0f;
            jointCmd_.kp = kJointKp;
            jointCmd_.kd = kJointKd;
        }
        else
        {
            // 非 Stair：回到收起位
            jointCmd_.leftDeg = kJointFoldDeg;
            jointCmd_.rightDeg = kJointFoldDeg;
            jointCmd_.velDegPerS = 0.0f;
            jointCmd_.kp = kJointFoldKp;
            jointCmd_.kd = kJointFoldKd;
        }
    }

    /* ================================================================
     * resolveYawCmd
     * ================================================================ */
    void RobotController::resolveYawCmd()
    {
        if (state_ != RobotState::Grab)
            return;

        targetYawDeg_ += grabLeftHArb_ * kYawStepDeg;
        targetYawDeg_ = clampF(targetYawDeg_, -kYawMaxDeg, kYawMaxDeg);
    }

    /* ================================================================
     * resolveLiftCmd
     * ================================================================ */
    void RobotController::resolveLiftCmd()
    {
        if (state_ != RobotState::Grab)
            return;

        lift_.updateTargetByJoystick(grabLeftVArb_, kTaskPeriodS);
    }

    /* ================================================================
     * resolveServoCmd
     * ================================================================ */
    void RobotController::resolveServoCmd()
    {
        if (state_ != RobotState::Grab)
            return;

        const RemoteData &rc = remote_.data();

        float dial = applyDeadband(rc.dial, kDialGripperThresh);

        if (dial > 0.0f)
        {
            gripperState_ = GripperState::Closed;
            servoCmd_.gripperUs = kGripperCloseUs;
        }
        else if (dial < 0.0f)
        {
            gripperState_ = GripperState::Open;
            servoCmd_.gripperUs = kGripperOpenUs;
        }
        // dial == 0：保持上一次状态不变
    }

    /* ================================================================
     * applyJointMotors
     * 只计算 MIT 目标，不发送 CAN（由 sendMotorCommands 统一发送）
     * ================================================================ */
    void RobotController::applyJointMotors()
    {
        joint_.setMitTarget(
            DmIndex::Left,
            jointCmd_.leftDeg,
            jointCmd_.velDegPerS,
            0.0f,
            jointCmd_.kp,
            jointCmd_.kd);

        joint_.setMitTarget(
            DmIndex::Right,
            jointCmd_.rightDeg,
            jointCmd_.velDegPerS,
            0.0f,
            jointCmd_.kp,
            jointCmd_.kd);
    }

    /* ================================================================
     * applyYawMotor
     * 只计算输出，不发送 CAN（由 sendMotorCommands 统一发送）
     * ================================================================ */
    void RobotController::applyYawMotor()
    {
        if (state_ == RobotState::Grab && yaw_.isOnline())
        {
            yaw_.setTargetAngleDeg(targetYawDeg_);
            yaw_.calcOutput();
            return;
        }

        // 非 Grab：同步目标到当前，清零输出
        yaw_.syncTargetToCurrent();
        targetYawDeg_ = yaw_.currentAngleDeg();
        yaw_.clearOutput();
    }

    /* ================================================================
     * applyLiftMotor
     * 只计算输出，不发送 CAN（由 sendMotorCommands 统一发送）
     * ================================================================ */
    void RobotController::applyLiftMotor()
    {
        lift_.calcOutput();
    }

    /* ================================================================
     * applyServos
     * PWM 直接写寄存器，不经过 CAN
     * ================================================================ */
    void RobotController::applyServos()
    {
        setServoPulse(kServoFoldCh, servoCmd_.foldUs);
        setServoPulse(kServoGripperCh, servoCmd_.gripperUs);
    }

    /* ================================================================
     * setServoPulse
     * ================================================================ */
    void RobotController::setServoPulse(uint32_t channel, uint32_t pulseUs)
    {
        pulseUs = clampPulse(pulseUs, kServoPwmMinUs, kServoPwmMaxUs);

        // 只在值变化时才写寄存器，避免每周期重复写入产生毛刺
        const uint32_t current = __HAL_TIM_GET_COMPARE(htim1_, channel);
        if (current == pulseUs)
            return;

        __HAL_TIM_SET_COMPARE(htim1_, channel, pulseUs);
    }

} // namespace Robot