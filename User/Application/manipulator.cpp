#include "manipulator.hpp"

#include "debug.hpp"

#include <cmath>

namespace Robot
{

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

void RobotController::resolveJointCmd()
    {
        /* 默认没有关节速度前馈，Stair分支按需要覆盖 */
        jointCmd_.velDegPerS = 0.0f;
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

void RobotController::resolveLiftCmd()
    {
        if (state_ == RobotState::Grab)
        {
            lift_.updateTargetByJoystick(grabLeftVArb_, kTaskPeriodS);
        }
        // 其余模式不调用 updateTargetByJoystick，目标保持不变
    }

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

void RobotController::applyLiftMotor()
    {
        // 升降电机的 CAN 帧已在 applyChassisMotors() 内与副履带合并发送
        // 此函数保留供后续扩展（如独立发送、状态检查等）
    }

void RobotController::applyServos()
    {
        setServoPulse(kServoFoldCh, servoCmd_.foldUs);
        setServoPulse(kServoGripperCh, servoCmd_.gripperUs);
    }

void RobotController::setServoPulse(uint32_t channel, uint32_t pulseUs)
    {
        pulseUs = clampPulse(pulseUs, kServoPwmMinUs, kServoPwmMaxUs);
        __HAL_TIM_SET_COMPARE(htim1_, channel, pulseUs);
    }

} // namespace Robot
