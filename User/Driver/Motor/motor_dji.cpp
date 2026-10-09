/**
 * @file    motor_dji.cpp
 * @brief   大疆电机驱动模块实现
 */

#include "motor_dji.hpp"
#include "bsp_can.hpp"
#include <cstring>
#include <cmath>

// debugChTx201 / debugChTx202 的赋值已移至
// communication.cpp 的 sendMotorCommands()，此处不再引用。

namespace Robot
{
    /* ================================================================
     * DjiMotorInstance
     * ================================================================ */

    void DjiMotorInstance::init(uint32_t feedbackId,
                                uint32_t commandId,
                                uint8_t slot,
                                DjiMotorType type,
                                const PidConfig &speedCfg,
                                const PidConfig *posCfg)
    {
        feedbackId_ = feedbackId;
        commandId_ = commandId;
        slot_ = slot;
        type_ = type;
        speedPid.init(speedCfg);
        if (posCfg != nullptr)
        {
            posPid.init(*posCfg);
        }
    }

    void DjiMotorInstance::updateFeedback(const uint8_t *pData)
    {
        if (pData == nullptr)
            return;

        lastUpdateMs_ = HAL_GetTick();
        online_ = true;

        uint16_t newEncode = static_cast<uint16_t>(
            (static_cast<uint16_t>(pData[0]) << 8U) |
            static_cast<uint16_t>(pData[1]));

        fb_.speedRpm = static_cast<int16_t>(
            (static_cast<uint16_t>(pData[2]) << 8U) |
            static_cast<uint16_t>(pData[3]));

        fb_.torqueCurrent = static_cast<int16_t>(
            (static_cast<uint16_t>(pData[4]) << 8U) |
            static_cast<uint16_t>(pData[5]));

        fb_.temperature = pData[6];

        if (!angleInited_)
        {
            fb_.angleRaw = newEncode;
            lastEncodeRaw_ = newEncode;
            encodeOffset_ = newEncode;
            roundCount_ = 0;
            angleDeg_ = 0.0f;
            angleInited_ = true;
            return;
        }

        fb_.angleRaw = newEncode;

        int32_t delta = static_cast<int32_t>(newEncode) -
                        static_cast<int32_t>(lastEncodeRaw_);
        if (delta > 4096)
            roundCount_--;
        if (delta < -4096)
            roundCount_++;

        angleDeg_ = (static_cast<float>(newEncode) +
                     static_cast<float>(roundCount_) * 8192.0f -
                     static_cast<float>(encodeOffset_)) /
                    8192.0f * 360.0f;

        lastEncodeRaw_ = newEncode;
    }

    void DjiMotorInstance::updateOnlineStatus(uint32_t nowMs)
    {
        if ((nowMs - lastUpdateMs_) > kDjiOfflineTimeoutMs)
        {
            online_ = false;
        }
    }

    void DjiMotorInstance::resetAngle()
    {
        angleInited_ = false;
    }

    /* ================================================================
     * DjiSendCanFrame
     * 负责 DJI 协议编码（int16_t → 8字节），底层调用 BspCanSendStdFrame
     * ================================================================ */

    bool DjiSendCanFrame(CAN_HandleTypeDef *hcan,
                         uint32_t cmdId,
                         const int16_t *outputs,
                         uint8_t count)
    {
        if (outputs == nullptr || count == 0U || count > 4U)
            return false;

        uint8_t txData[8] = {};

        for (uint8_t i = 0U; i < count; ++i)
        {
            // 用无符号位模式拆高低字节，避免有符号右移实现依赖
            const uint16_t raw = static_cast<uint16_t>(outputs[i]);
            txData[i * 2U] = static_cast<uint8_t>(raw >> 8U);
            txData[i * 2U + 1U] = static_cast<uint8_t>(raw & 0xFFU);
        }

        return BspCanSendStdFrame(hcan, cmdId, txData, 8U);
    }

    /* ================================================================
     * ChassisMotorController（M3508）
     * ================================================================ */

    void ChassisMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        PidConfig speedCfg;
        speedCfg.kp = 10.0f;
        speedCfg.ki = 0.0f;
        speedCfg.kd = 0.0f;
        speedCfg.outputLimit = static_cast<float>(kM3508MaxCurrent);
        speedCfg.integralLimit = 4000.0f;
        speedCfg.mode = PidMode::Position;

        for (uint8_t i = 0U; i < kChassisMotorCount; i++)
        {
            motors_[i].init(kChassisFeedbackBase + i,
                            kChassisCmdId, i,
                            DjiMotorType::M3508, speedCfg);
        }
    }

    void ChassisMotorController::setTargetSpeedRpm(uint8_t idx, float rpm)
    {
        if (idx >= kChassisMotorCount)
            return;
        motors_[idx].targetSpeedRpm = rpm;
    }

    void ChassisMotorController::calcOutput()
    {
        // 任意一台离线则两侧均禁止，避免单侧驱动导致意外转向
        // if (!isAllOnline())
        // {
        //     clearOutput();
        //     return;
        // }

        for (uint8_t i = 0U; i < kChassisMotorCount; ++i)
        {
            DjiMotorInstance &m = motors_[i];

            if (!std::isfinite(m.targetSpeedRpm))
            {
                m.speedPid.reset();
                m.targetOutput = 0;
                continue;
            }

            const float out = m.speedPid.update(
                m.targetSpeedRpm,
                static_cast<float>(m.feedback().speedRpm));

            m.targetOutput = DjiOutputFromFloat(out, kM3508MaxCurrent);
        }
    }

    int16_t ChassisMotorController::getTargetOutput(uint8_t idx) const
    {
        if (idx >= kChassisMotorCount)
            return 0;
        return motors_[idx].targetOutput;
    }

    void ChassisMotorController::clearOutput()
    {
        for (uint8_t i = 0U; i < kChassisMotorCount; ++i)
        {
            motors_[i].targetSpeedRpm = 0.0f;
            motors_[i].targetOutput = 0;
            motors_[i].speedPid.reset();
        }
    }

    bool ChassisMotorController::sendAllCurrent()
    {
        calcOutput();

        int16_t outputs[4] = {};
        for (uint8_t i = 0U; i < kChassisMotorCount; ++i)
        {
            outputs[i] = getTargetOutput(i);
        }

        return DjiSendCanFrame(hcan_, kChassisCmdId, outputs, 4U);
    }

    void ChassisMotorController::updateFeedback(uint32_t canId,
                                                const uint8_t *pData)
    {
        for (uint8_t i = 0U; i < kChassisMotorCount; i++)
        {
            if (motors_[i].feedbackId() == canId)
            {
                motors_[i].updateFeedback(pData);
                return;
            }
        }
    }

    void ChassisMotorController::updateOnlineStatus()
    {
        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0U; i < kChassisMotorCount; i++)
            motors_[i].updateOnlineStatus(now);
    }

    bool ChassisMotorController::isAllOnline() const
    {
        for (uint8_t i = 0U; i < kChassisMotorCount; i++)
            if (!motors_[i].isOnline())
                return false;
        return true;
    }

    const DjiMotorInstance *ChassisMotorController::getInstance(
        uint8_t idx) const
    {
        if (idx >= kChassisMotorCount)
            return nullptr;
        return &motors_[idx];
    }

    /* ================================================================
     * SubTrackMotorController（M2006 副履带）
     * ================================================================ */

    void SubTrackMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        PidConfig speedCfg;
        speedCfg.kp = 2.0f;
        speedCfg.ki = 0.0f;
        speedCfg.kd = 0.0f;
        speedCfg.outputLimit = static_cast<float>(kM2006MaxCurrent);
        speedCfg.integralLimit = 0.0f;
        speedCfg.mode = PidMode::Position;

        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
        {
            motors_[i].init(kSubTrackFeedbackBase + i,
                            kSubTrackCmdId, i,
                            DjiMotorType::M2006, speedCfg);
        }
    }

    void SubTrackMotorController::setTargetSpeedRpm(uint8_t idx, float rpm)
    {
        if (idx >= kSubTrackMotorCount)
            return;
        motors_[idx].targetSpeedRpm = rpm;
    }

    void SubTrackMotorController::calcOutput()
    {
        for (uint8_t i = 0U; i < kSubTrackMotorCount; ++i)
        {
            DjiMotorInstance &m = motors_[i];

            if (!m.isOnline() || !std::isfinite(m.targetSpeedRpm))
            {
                m.speedPid.reset();
                m.targetOutput = 0;
                continue;
            }

            const float out = m.speedPid.update(
                m.targetSpeedRpm,
                static_cast<float>(m.feedback().speedRpm));

            m.targetOutput = DjiOutputFromFloat(out, kM2006MaxCurrent);
        }
    }

    void SubTrackMotorController::resetPid()
    {
        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
            motors_[i].speedPid.reset();
    }

    void SubTrackMotorController::clearOutput()
    {
        for (uint8_t i = 0U; i < kSubTrackMotorCount; ++i)
        {
            motors_[i].targetSpeedRpm = 0.0f;
            motors_[i].targetOutput = 0;
            motors_[i].speedPid.reset();
        }
    }

    int16_t SubTrackMotorController::getTargetOutput(uint8_t idx) const
    {
        if (idx >= kSubTrackMotorCount)
            return 0;
        return motors_[idx].targetOutput;
    }

    const DjiMotorInstance *SubTrackMotorController::getInstance(
        uint8_t idx) const
    {
        if (idx >= kSubTrackMotorCount)
            return nullptr;
        return &motors_[idx];
    }

    void SubTrackMotorController::updateFeedback(uint32_t canId,
                                                 const uint8_t *pData)
    {
        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
        {
            if (motors_[i].feedbackId() == canId)
            {
                motors_[i].updateFeedback(pData);
                return;
            }
        }
    }

    void SubTrackMotorController::updateOnlineStatus()
    {
        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
            motors_[i].updateOnlineStatus(now);
    }

    /* ================================================================
     * LiftMotorController（M2006 升降）
     * ================================================================ */

    void LiftMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        PidConfig speedCfg;
        speedCfg.kp = 8.0f;
        speedCfg.ki = 0.0f;
        speedCfg.kd = 0.0f;
        speedCfg.outputLimit = static_cast<float>(kM2006MaxCurrent);
        speedCfg.integralLimit = 2000.0f;
        speedCfg.mode = PidMode::Position;

        motor_.init(kLiftFeedbackId,
                    kSubTrackCmdId,
                    2U,
                    DjiMotorType::M2006,
                    speedCfg);
    }

    void LiftMotorController::setTargetPosition(float posCnt)
    {
        targetPosCnt_ = posCnt;
    }

    float LiftMotorController::currentPosition() const
    {
        return motor_.angleDeg() / 360.0f * 8192.0f;
    }

    void LiftMotorController::updateTargetByJoystick(float normalizedInput,
                                                     float dtSeconds)
    {
        if (fabsf(normalizedInput) > 0.01f)
        {
            targetPosCnt_ += normalizedInput * ratePerSec_ * dtSeconds;
        }

        constexpr float kLiftPosMin = -20000.0f;
        constexpr float kLiftPosMax = 20000.0f;
        if (targetPosCnt_ < kLiftPosMin)
            targetPosCnt_ = kLiftPosMin;
        if (targetPosCnt_ > kLiftPosMax)
            targetPosCnt_ = kLiftPosMax;
    }

    void LiftMotorController::calcOutput()
    {
        // 反馈离线时不继续使用旧反馈计算输出。
        // 注意：清零电流不能保证垂直机构不下落，需有机械支撑/制动。
        if (!motor_.isOnline())
        {
            motor_.targetOutput = 0;
            motor_.speedPid.reset();
            return;
        }

        const float currentPos = currentPosition();

        if (!std::isfinite(targetPosCnt_) ||
            !std::isfinite(currentPos))
        {
            motor_.targetOutput = 0;
            motor_.speedPid.reset();
            return;
        }

        const float posErr = targetPosCnt_ - currentPos;

        // 外环：位置误差转换为目标转子速度，单位 rpm。
        float targetRpm = posKp_ * posErr;

        if (targetRpm > maxRpm_)
            targetRpm = maxRpm_;

        if (targetRpm < -maxRpm_)
            targetRpm = -maxRpm_;

        motor_.targetSpeedRpm = targetRpm;

        // 内环：即使接近目标，也持续控制实际速度。
        const float out = motor_.speedPid.update(
            targetRpm,
            static_cast<float>(motor_.feedback().speedRpm));

        // 重力前馈与闭环纠偏同时起作用。
        motor_.targetOutput = DjiOutputFromFloat(
            out + static_cast<float>(kLiftGravityFF),
            kM2006MaxCurrent);
    }

    int16_t LiftMotorController::getTargetOutput() const
    {
        return motor_.targetOutput;
    }

    bool LiftMotorController::isSafeToFold(float safePos,
                                           float thresh) const
    {
        return fabsf(currentPosition() - safePos) <= thresh;
    }

    void LiftMotorController::updateFeedback(uint32_t canId,
                                             const uint8_t *pData)
    {
        if (motor_.feedbackId() == canId)
        {
            motor_.updateFeedback(pData);
        }
    }

    void LiftMotorController::updateOnlineStatus()
    {
        motor_.updateOnlineStatus(HAL_GetTick());
    }

    bool LiftMotorController::isOnline() const
    {
        return motor_.isOnline();
    }

    void LiftMotorController::syncTargetToCurrent()
    {
        targetPosCnt_ = currentPosition();
        motor_.speedPid.reset();
    }

    /* ================================================================
     * YawMotorController（GM6020）
     * ================================================================ */

    void YawMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        PidConfig posCfg;
        posCfg.kp = 8.0f;
        posCfg.ki = 0.0f;
        posCfg.kd = 0.1f;
        posCfg.outputLimit = 200.0f;
        posCfg.integralLimit = 0.0f;
        posCfg.mode = PidMode::Position;

        PidConfig speedCfg;
        speedCfg.kp = 10.0f;
        speedCfg.ki = 0.0f;
        speedCfg.kd = 0.0f;
        speedCfg.outputLimit = static_cast<float>(kGm6020MaxCurrent);
        speedCfg.integralLimit = 8000.0f;
        speedCfg.mode = PidMode::Position;

        motor_.init(kYawFeedbackId, kYawCmdId, kYawSlot,
                    DjiMotorType::Gm6020, speedCfg, &posCfg);
        targetAngleDeg_ = 0.0f;
    }

    void YawMotorController::setTargetAngleDeg(float angleDeg)
    {
        targetAngleDeg_ = angleDeg;
    }

    void YawMotorController::syncTargetToCurrent()
    {
        targetAngleDeg_ = motor_.angleDeg();
        motor_.speedPid.reset();
        motor_.posPid.reset();
    }

    void YawMotorController::calcOutput()
    {
        if (!motor_.isOnline() || !std::isfinite(targetAngleDeg_))
        {
            clearOutput();
            return;
        }
        runCascadePid();
    }

    int16_t YawMotorController::getTargetOutput() const
    {
        return motor_.targetOutput;
    }

    void YawMotorController::clearOutput()
    {
        motor_.targetOutput = 0;
        motor_.speedPid.reset();
        motor_.posPid.reset();
    }

    // 兼容接口：迁移完成后删除
    bool YawMotorController::sendCurrent()
    {
        calcOutput();

        int16_t outputs[4] = {};
        outputs[kYawSlot] = getTargetOutput();

        return DjiSendCanFrame(hcan_, kYawCmdId, outputs, 4U);
    }

    // 兼容接口：迁移完成后删除
    bool YawMotorController::sendZeroCurrent()
    {
        clearOutput();

        const int16_t outputs[4] = {};
        return DjiSendCanFrame(hcan_, kYawCmdId, outputs, 4U);
    }

    float YawMotorController::currentAngleDeg() const
    {
        return motor_.angleDeg();
    }

    void YawMotorController::updateFeedback(uint32_t canId,
                                            const uint8_t *pData)
    {
        if (motor_.feedbackId() == canId)
            motor_.updateFeedback(pData);
    }

    void YawMotorController::updateOnlineStatus()
    {
        motor_.updateOnlineStatus(HAL_GetTick());
    }

    bool YawMotorController::isOnline() const
    {
        return motor_.isOnline();
    }

    void YawMotorController::runCascadePid()
    {
        constexpr float kDeadband = 1.0f;
        const float err = targetAngleDeg_ - motor_.angleDeg();

        if (fabsf(err) < kDeadband)
        {
            clearOutput();
            return;
        }

        const float targetRpm = motor_.posPid.update(
            targetAngleDeg_, motor_.angleDeg());

        const float out = motor_.speedPid.update(
            targetRpm,
            static_cast<float>(motor_.feedback().speedRpm));

        motor_.targetOutput = DjiOutputFromFloat(out, kGm6020MaxCurrent);
    }

} // namespace Robot