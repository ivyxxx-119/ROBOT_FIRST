/**
 * @file    motor_dm.cpp
 * @brief   达妙电机驱动模块实现（C++）
 */

#include "motor_dm.hpp"
#include "bsp_can.hpp"
#include <cstring>
#include <cmath>

namespace Robot
{
    /* ================================================================
     * DmMotorInstance
     * ================================================================ */

    void DmMotorInstance::init(uint32_t txCanId,
                               uint32_t rxCanId,
                               float kp_,
                               float kd_)
    {
        txCanId_ = txCanId;
        rxCanId_ = rxCanId;
        kp = kp_;
        kd = kd_;
        enabled_ = false;
        online_ = false;
    }

    void DmMotorInstance::updateOnlineStatus(uint32_t nowMs)
    {
        if ((nowMs - lastUpdateMs_) > kDmOfflineTimeoutMs)
        {
            online_ = false;
        }
    }

    void DmMotorInstance::parseFeedback(const uint8_t *pData)
    {
        fb_.motorId = pData[0] & 0x0FU;
        fb_.errorCode = pData[0] >> 4U;

        const uint16_t posRaw = static_cast<uint16_t>(
            (static_cast<uint16_t>(pData[1]) << 8U) |
            static_cast<uint16_t>(pData[2]));

        const uint16_t velRaw = static_cast<uint16_t>(
            (static_cast<uint16_t>(pData[3]) << 4U) |
            (static_cast<uint16_t>(pData[4]) >> 4U));

        const uint16_t torRaw = static_cast<uint16_t>(
            ((static_cast<uint16_t>(pData[4]) & 0x0FU) << 8U) |
            static_cast<uint16_t>(pData[5]));

        const float angleRad = uintToFloat(
            posRaw, -kDm4310PosMaxRad, kDm4310PosMaxRad, 16U);

        const float velocityRadPerS = uintToFloat(
            velRaw, -kDm4310VelMaxRadPerS, kDm4310VelMaxRadPerS, 12U);

        fb_.angleDeg = angleRad * kDmRadToDeg;
        fb_.velocityDegPerS = velocityRadPerS * kDmRadToDeg;
        fb_.torque = uintToFloat(
            torRaw, -kDm4310TorqueMax, kDm4310TorqueMax, 12U);
    }

    float DmMotorInstance::uintToFloat(uint32_t raw,
                                       float minVal,
                                       float maxVal,
                                       uint8_t bits)
    {
        const uint32_t maxRaw = (1UL << bits) - 1UL;
        return minVal +
               static_cast<float>(raw) *
                   (maxVal - minVal) /
                   static_cast<float>(maxRaw);
    }

    /* ================================================================
     * DmMotorController
     * ================================================================ */

    void DmMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        motors_[static_cast<uint8_t>(DmIndex::Left)].init(
            kDmJointLeftTxCanId, kDmJointLeftRxCanId, 10.0f, 1.0f);

        motors_[static_cast<uint8_t>(DmIndex::Right)].init(
            kDmJointRightTxCanId, kDmJointRightRxCanId, 10.0f, 1.0f);
    }

    void DmMotorController::enable(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
            return;

        if (sendSpecialCmd(m->txCanId(), kDmEnableCmd))
        {
            m->setEnabled(true);
        }
    }

    void DmMotorController::disable(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
            return;

        if (sendSpecialCmd(m->txCanId(), kDmDisableCmd))
        {
            m->setEnabled(false);
        }
    }

    void DmMotorController::clearFault(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
            return;

        sendSpecialCmd(m->txCanId(), kDmClearFaultCmd);
    }

    void DmMotorController::setMitTarget(DmIndex idx,
                                         float targetAngleDeg,
                                         float targetVelDegPerS,
                                         float targetTorque,
                                         float kp,
                                         float kd)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
            return;

        m->targetAngleDeg = targetAngleDeg;
        m->targetVelDegPerS = targetVelDegPerS;
        m->targetTorque = targetTorque;
        m->kp = kp;
        m->kd = kd;
    }

    bool DmMotorController::buildMitFrame(DmIndex idx,
                                          DmMitFrame &frame) const
    {
        frame = DmMitFrame{};

        const DmMotorInstance *m = getInstance(idx);
        if (m == nullptr || !m->isEnabled())
            return false;

        // 检查所有目标值是有限浮点数
        if (!std::isfinite(m->targetAngleDeg) ||
            !std::isfinite(m->targetVelDegPerS) ||
            !std::isfinite(m->targetTorque) ||
            !std::isfinite(m->kp) ||
            !std::isfinite(m->kd))
        {
            return false;
        }

        const float posRad = m->targetAngleDeg * kDmDegToRad;
        const float velRadPerS = m->targetVelDegPerS * kDmDegToRad;

        if (!std::isfinite(posRad) || !std::isfinite(velRadPerS))
            return false;

        const uint32_t posU = floatToUint(
            posRad, -kDm4310PosMaxRad, kDm4310PosMaxRad, 16U);

        const uint32_t velU = floatToUint(
            velRadPerS, -kDm4310VelMaxRadPerS, kDm4310VelMaxRadPerS, 12U);

        const uint32_t kpU = floatToUint(
            m->kp, 0.0f, kDm4310KpMax, 12U);

        const uint32_t kdU = floatToUint(
            m->kd, 0.0f, kDm4310KdMax, 12U);

        const uint32_t torU = floatToUint(
            m->targetTorque, -kDm4310TorqueMax, kDm4310TorqueMax, 12U);

        frame.canId = m->txCanId();
        frame.data[0] = static_cast<uint8_t>(posU >> 8U);
        frame.data[1] = static_cast<uint8_t>(posU & 0xFFU);
        frame.data[2] = static_cast<uint8_t>(velU >> 4U);
        frame.data[3] = static_cast<uint8_t>(
            ((velU & 0x0FU) << 4U) | ((kpU >> 8U) & 0x0FU));
        frame.data[4] = static_cast<uint8_t>(kpU & 0xFFU);
        frame.data[5] = static_cast<uint8_t>(kdU >> 4U);
        frame.data[6] = static_cast<uint8_t>(
            ((kdU & 0x0FU) << 4U) | ((torU >> 8U) & 0x0FU));
        frame.data[7] = static_cast<uint8_t>(torU & 0xFFU);
        frame.valid = true;

        return true;
    }

    // 兼容接口：迁移完成后删除
    bool DmMotorController::sendMitCommand(DmIndex idx)
    {
        DmMitFrame frame{};
        if (!buildMitFrame(idx, frame))
            return false;

        return sendRawFrame(frame.canId, frame.data, 8U);
    }

    void DmMotorController::updateFeedback(uint32_t canId,
                                           const uint8_t *pData,
                                           uint8_t dlc)
    {
        if (pData == nullptr || dlc != 8U)
            return;

        for (uint8_t i = 0U; i < kDmMotorCount; ++i)
        {
            DmMotorInstance &m = motors_[i];
            if (m.rxCanId() == canId)
            {
                m.parseFeedback(pData);
                m.setOnline(true);
                m.updateLastMs(HAL_GetTick());
                break;
            }
        }
    }

    void DmMotorController::updateOnlineStatus()
    {
        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0U; i < kDmMotorCount; i++)
            motors_[i].updateOnlineStatus(now);
    }

    const DmMotorInstance *DmMotorController::getInstance(
        DmIndex idx) const
    {
        uint8_t i = static_cast<uint8_t>(idx);
        if (i >= kDmMotorCount)
            return nullptr;
        return &motors_[i];
    }

    uint32_t DmMotorController::floatToUint(float v,
                                            float vMin,
                                            float vMax,
                                            uint8_t bits)
    {
        if (v < vMin)
            v = vMin;
        if (v > vMax)
            v = vMax;

        const uint32_t maxU = (1UL << bits) - 1UL;
        return static_cast<uint32_t>(
            (v - vMin) * static_cast<float>(maxU) / (vMax - vMin));
    }

    bool DmMotorController::sendRawFrame(uint32_t canId,
                                         const uint8_t *pData,
                                         uint8_t len)
    {
        return BspCanSendStdFrame(hcan_, canId, pData, len);
    }

    bool DmMotorController::sendSpecialCmd(uint32_t motorCanId,
                                           uint8_t cmd)
    {
        const uint8_t tx[8] = {
            0xFFU, 0xFFU, 0xFFU, 0xFFU,
            0xFFU, 0xFFU, 0xFFU, cmd};
        return sendRawFrame(motorCanId, tx, 8U);
    }

    DmMotorInstance *DmMotorController::getMotor(DmIndex idx)
    {
        uint8_t i = static_cast<uint8_t>(idx);
        if (i >= kDmMotorCount)
            return nullptr;
        return &motors_[i];
    }

} // namespace Robot