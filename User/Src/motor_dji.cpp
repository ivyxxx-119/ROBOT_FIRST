/**
 * @file    motor_dji.cpp
 * @brief   大疆电机驱动模块实现
 */

#include "motor_dji.hpp"
#include <cstring>
#include <cmath>

extern volatile int16_t debugChTx201;
extern volatile int16_t debugChTx202;

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
        else if (delta < -4096)
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
     * ================================================================ */

    bool DjiSendCanFrame(CAN_HandleTypeDef *hcan,
                         uint32_t cmdId,
                         const int16_t *outputs,
                         uint8_t count)
    {
        if (hcan == nullptr || outputs == nullptr)
            return false;

        // 等待空闲邮箱，超时 1 ms 放弃（同一总线高负载时保护）
        uint32_t start = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
        {
            if ((HAL_GetTick() - start) >= 1U)
                return false;
        }

        CAN_TxHeaderTypeDef txHeader{};
        txHeader.StdId = cmdId;
        txHeader.IDE = CAN_ID_STD;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = 8U;
        txHeader.TransmitGlobalTime = DISABLE;

        uint8_t txData[8] = {0U};
        uint32_t mailbox = 0U;
        uint8_t n = (count < 4U) ? count : 4U;

        for (uint8_t i = 0U; i < n; i++)
        {
            txData[i * 2U] = static_cast<uint8_t>(outputs[i] >> 8);
            txData[i * 2U + 1U] = static_cast<uint8_t>(outputs[i] & 0xFF);
        }

        return HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &mailbox) == HAL_OK;
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

    bool ChassisMotorController::sendAllCurrent()
    {
        int16_t outputs[kChassisMotorCount];
        for (uint8_t i = 0U; i < kChassisMotorCount; i++)
        {
            float out = motors_[i].speedPid.update(
                motors_[i].targetSpeedRpm,
                static_cast<float>(motors_[i].feedback().speedRpm));
            motors_[i].targetOutput = DjiClampOutput(
                static_cast<int16_t>(out), kM3508MaxCurrent);
            outputs[i] = motors_[i].targetOutput;
        }
        debugChTx201 = outputs[0];
        debugChTx202 = outputs[1];
        return DjiSendCanFrame(hcan_, kChassisCmdId, outputs, kChassisMotorCount);
    }

    void ChassisMotorController::updateFeedback(uint32_t canId, const uint8_t *pData)
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

    const DjiMotorInstance *ChassisMotorController::getInstance(uint8_t idx) const
    {
        if (idx >= kChassisMotorCount)
            return nullptr;
        return &motors_[idx];
    }

    /* ================================================================
     * SubTrackMotorController（M2006 副履带）
     * 只计算 targetOutput，不独立发送 CAN 帧
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
        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
        {
            float out = motors_[i].speedPid.update(
                motors_[i].targetSpeedRpm,
                static_cast<float>(motors_[i].feedback().speedRpm));
            motors_[i].targetOutput = DjiClampOutput(
                static_cast<int16_t>(out), kM2006MaxCurrent);
        }
    }

    void SubTrackMotorController::resetPid()
    {
        for (uint8_t i = 0U; i < kSubTrackMotorCount; i++)
            motors_[i].speedPid.reset();
    }

    int16_t SubTrackMotorController::getTargetOutput(uint8_t idx) const
    {
        if (idx >= kSubTrackMotorCount)
            return 0;
        return motors_[idx].targetOutput;
    }

    const DjiMotorInstance *SubTrackMotorController::getInstance(uint8_t idx) const
    {
        if (idx >= kSubTrackMotorCount)
            return nullptr;
        return &motors_[idx];
    }

    void SubTrackMotorController::updateFeedback(uint32_t canId, const uint8_t *pData)
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
     * 与副履带共用 0x1FF 命令帧的 slot2
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

        // 升降电机：反馈 0x207，命令帧 0x1FF，slot 2
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
        // 用多圈角度换算为编码器 count（1圈=8192 count，360°=8192 count）
        // angleDeg_ 是相对上电零点的多圈度数
        return motor_.angleDeg() / 360.0f * 8192.0f;
    }

    void LiftMotorController::updateTargetByJoystick(float normalizedInput,
                                                     float dtSeconds)
    {
        // 摇杆回中（normalizedInput≈0）时不更新目标，保持当前高度
        if (fabsf(normalizedInput) > 0.01f)
        {
            targetPosCnt_ += normalizedInput * ratePerSec_ * dtSeconds;
        }
        // 目标限幅（此处使用示例范围，需按实际行程标定）
        // 正方向为上升，需实测确认编码器方向
        constexpr float kLiftPosMin = -20000.0f; // 需实测
        constexpr float kLiftPosMax = 20000.0f;  // 需实测
        if (targetPosCnt_ < kLiftPosMin)
            targetPosCnt_ = kLiftPosMin;
        if (targetPosCnt_ > kLiftPosMax)
            targetPosCnt_ = kLiftPosMax;
    }

    void LiftMotorController::calcOutput()
    {
        // 外环：位置 → 目标转速
        float posErr = targetPosCnt_ - currentPosition();

        // 死区：误差小于 50 count 时只保持重力补偿，不运行 PID
        if (fabsf(posErr) < 50.0f)
        {
            motor_.targetOutput = kLiftGravityFF;
            motor_.speedPid.reset();
            return;
        }

        float targetRpm = posKp_ * posErr;
        if (targetRpm > maxRpm_)
            targetRpm = maxRpm_;
        if (targetRpm < -maxRpm_)
            targetRpm = -maxRpm_;

        // 内环：转速 PID → 电流，叠加重力补偿前馈
        float out = motor_.speedPid.update(
            targetRpm,
            static_cast<float>(motor_.feedback().speedRpm));
        motor_.targetOutput = DjiClampOutput(
            static_cast<int16_t>(out) + kLiftGravityFF, kM2006MaxCurrent);
    }

    int16_t LiftMotorController::getTargetOutput() const
    {
        return motor_.targetOutput;
    }

    bool LiftMotorController::isSafeToFold(float safePos, float thresh) const
    {
        return fabsf(currentPosition() - safePos) <= thresh;
    }

    void LiftMotorController::updateFeedback(uint32_t canId, const uint8_t *pData)
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

    bool YawMotorController::sendCurrent()
    {
        if (!motor_.isOnline())
        {
            motor_.targetOutput = 0;
            const int16_t outputs[4] = {0};
            return DjiSendCanFrame(hcan_, kYawCmdId, outputs, 4U);
        }
        runCascadePid();
        int16_t outputs[4] = {0};
        outputs[kYawSlot] = DjiClampOutput(motor_.targetOutput, kGm6020MaxCurrent);
        return DjiSendCanFrame(hcan_, kYawCmdId, outputs, 4U);
    }

    bool YawMotorController::sendZeroCurrent()
    {
        motor_.targetOutput = 0;
        const int16_t outputs[4] = {0};
        return DjiSendCanFrame(hcan_, kYawCmdId, outputs, 4U);
    }

    float YawMotorController::currentAngleDeg() const
    {
        return motor_.angleDeg();
    }

    void YawMotorController::updateFeedback(uint32_t canId, const uint8_t *pData)
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
        float err = targetAngleDeg_ - motor_.angleDeg();
        if (fabsf(err) < kDeadband)
        {
            motor_.targetOutput = 0;
            motor_.speedPid.reset();
            motor_.posPid.reset();
            return;
        }
        float targetRpm = motor_.posPid.update(targetAngleDeg_, motor_.angleDeg());
        float out = motor_.speedPid.update(
            targetRpm, static_cast<float>(motor_.feedback().speedRpm));
        motor_.targetOutput = DjiClampOutput(
            static_cast<int16_t>(out), kGm6020MaxCurrent);
    }

} // namespace Robot