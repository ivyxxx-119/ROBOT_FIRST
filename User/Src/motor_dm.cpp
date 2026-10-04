/**
 * @file    motor_dm.cpp
 * @brief   达妙电机驱动模块实现（C++）
 *
 * 反馈帧布局（MIT模式，8字节）：
 *   Byte0       [7:4]=ErrorCode  [3:0]=MotorID
 *   Byte1~2     pos[15:0]
 *   Byte3[7:4]  vel[11:8]
 *   Byte3[3:0]  vel[7:4]（与Byte4高4位合并）
 *   Byte4[7:4]  vel[3:0]
 *   Byte4[3:0]  tor[11:8]
 *   Byte5       tor[7:0]
 *   Byte6       MOS温度
 *   Byte7       线圈温度
 */

#include "motor_dm.hpp"
#include <cstring>

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

        uint16_t posRaw = static_cast<uint16_t>(
            (static_cast<uint16_t>(pData[1]) << 8U) |
            static_cast<uint16_t>(pData[2]));

        uint16_t velRaw = static_cast<uint16_t>(
            (static_cast<uint16_t>(pData[3]) << 4U) |
            (static_cast<uint16_t>(pData[4]) >> 4U));

        uint16_t torRaw = static_cast<uint16_t>(
            ((static_cast<uint16_t>(pData[4]) & 0x0FU) << 8U) |
            static_cast<uint16_t>(pData[5]));

        const float angleRad = uintToFloat(
            posRaw,
            -kDm4310PosMaxRad,
            kDm4310PosMaxRad,
            16U);

        const float velocityRadPerS = uintToFloat(
            velRaw,
            -kDm4310VelMaxRadPerS,
            kDm4310VelMaxRadPerS,
            12U);

        // 项目反馈结构仍用度、度/秒
        fb_.angleDeg = angleRad * kDmRadToDeg;
        fb_.velocityDegPerS = velocityRadPerS * kDmRadToDeg;

        fb_.torque = uintToFloat(
            torRaw,
            -kDm4310TorqueMax,
            kDm4310TorqueMax,
            12U);
    }

    float DmMotorInstance::uintToFloat(uint32_t raw,
                                       float minVal,
                                       float maxVal,
                                       uint8_t bits)
    {
        uint32_t maxRaw = (1UL << bits) - 1UL;
        return minVal + static_cast<float>(raw) * (maxVal - minVal) / static_cast<float>(maxRaw);
    }

    /* ================================================================
     * DmMotorController
     * ================================================================ */

    void DmMotorController::init(CAN_HandleTypeDef *hcan)
    {
        hcan_ = hcan;

        motors_[static_cast<uint8_t>(DmIndex::Left)].init(
            kDmJointLeftTxCanId,
            kDmJointLeftRxCanId,
            10.0f,
            1.0f);

        motors_[static_cast<uint8_t>(DmIndex::Right)].init(
            kDmJointRightTxCanId,
            kDmJointRightRxCanId,
            10.0f,
            1.0f);
    }

    void DmMotorController::enable(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
        {
            return;
        }

        if (sendSpecialCmd(static_cast<uint8_t>(m->txCanId()),
                           kDmEnableCmd))
        {
            m->setEnabled(true);
        }
    }

    void DmMotorController::disable(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
        {
            return;
        }
        if (sendSpecialCmd(static_cast<uint8_t>(m->txCanId()),
                           kDmDisableCmd))
        {
            m->setEnabled(false);
        }
    }

    void DmMotorController::clearFault(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr)
        {
            return;
        }
        sendSpecialCmd(static_cast<uint8_t>(m->txCanId()),
                       kDmClearFaultCmd);
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
        {
            return;
        }
        m->targetAngleDeg = targetAngleDeg;
        m->targetVelDegPerS = targetVelDegPerS;
        m->targetTorque = targetTorque;
        m->kp = kp;
        m->kd = kd;
    }

    bool DmMotorController::sendMitCommand(DmIndex idx)
    {
        DmMotorInstance *m = getMotor(idx);
        if (m == nullptr || !m->isEnabled())
            return false;

        const float targetPosRad = m->targetAngleDeg * kDmDegToRad;
        const float targetVelRadPerS = m->targetVelDegPerS * kDmDegToRad;

        uint32_t posU = floatToUint(targetPosRad, -kDm4310PosMaxRad, kDm4310PosMaxRad, 16U);
        uint32_t velU = floatToUint(targetVelRadPerS, -kDm4310VelMaxRadPerS, kDm4310VelMaxRadPerS, 12U);
        uint32_t kpU  = floatToUint(m->kp, 0.0f, kDm4310KpMax, 12U);
        uint32_t kdU  = floatToUint(m->kd, 0.0f, kDm4310KdMax, 12U);
        uint32_t torU = floatToUint(m->targetTorque, -kDm4310TorqueMax, kDm4310TorqueMax, 12U);

        // MIT控制帧格式（达妙DM4310）：
        // Byte0~1: pos[15:0]
        // Byte2:   vel[11:4]
        // Byte3:   vel[3:0]<<4 | kp[11:8]
        // Byte4:   kp[7:0]
        // Byte5:   kd[11:4]   （注：原始协议 kd 12bit，此处 Byte5 高8位）
        // Byte6:   kd[3:0]<<4 | tor[11:8]
        // Byte7:   tor[7:0]
        uint8_t tx[8];
        tx[0] = static_cast<uint8_t>(posU >> 8U);
        tx[1] = static_cast<uint8_t>(posU & 0xFFU);
        tx[2] = static_cast<uint8_t>(velU >> 4U);
        tx[3] = static_cast<uint8_t>(((velU & 0x0FU) << 4U) | (kpU >> 8U));
        tx[4] = static_cast<uint8_t>(kpU & 0xFFU);
        tx[5] = static_cast<uint8_t>(kdU >> 4U);
        tx[6] = static_cast<uint8_t>(((kdU & 0x0FU) << 4U) | (torU >> 8U));
        tx[7] = static_cast<uint8_t>(torU & 0xFFU);

        return sendRawFrame(m->txCanId(), tx, 8U);
    }

    void DmMotorController::updateFeedback(uint32_t canId,
                                           const uint8_t *pData,
                                           uint8_t dlc)
    {
        if (pData == nullptr || dlc != 8U)
        {
            return;
        }

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
        {
            motors_[i].updateOnlineStatus(now);
        }
    }

    const DmMotorInstance *DmMotorController::getInstance(DmIndex idx) const
    {
        uint8_t i = static_cast<uint8_t>(idx);
        if (i >= kDmMotorCount)
        {
            return nullptr;
        }
        return &motors_[i];
    }

    uint32_t DmMotorController::floatToUint(float v,
                                            float vMin,
                                            float vMax,
                                            uint8_t bits)
    {
        if (v < vMin)
        {
            v = vMin;
        }
        if (v > vMax)
        {
            v = vMax;
        }

        const uint32_t maxU = (1UL << bits) - 1UL;

        return static_cast<uint32_t>(
            (v - vMin) * static_cast<float>(maxU) / (vMax - vMin));
    }

    bool DmMotorController::sendRawFrame(uint32_t canId,
                                         const uint8_t *pData,
                                         uint8_t len)
    {
        if (hcan_ == nullptr)
            return false;

        uint32_t start = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0U)
        {
            if ((HAL_GetTick() - start) >= 1U)
                return false;
        }

        CAN_TxHeaderTypeDef hdr{};
        hdr.StdId = canId;
        hdr.IDE = CAN_ID_STD;
        hdr.RTR = CAN_RTR_DATA;
        hdr.DLC = len;
        hdr.TransmitGlobalTime = DISABLE;

        uint32_t mailbox = 0U;
        return HAL_CAN_AddTxMessage(
                   hcan_, &hdr, const_cast<uint8_t *>(pData), &mailbox) == HAL_OK;
    }

    bool DmMotorController::sendSpecialCmd(uint8_t motorCanId, uint8_t cmd)
    {
        uint8_t tx[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                         0xFFU, 0xFFU, 0xFFU, cmd};
        // enable/disable/clearFault 帧ID = 电机txCanId，Byte7 = 指令码
        return sendRawFrame(static_cast<uint32_t>(motorCanId), tx, 8U);
    }

    DmMotorInstance *DmMotorController::getMotor(DmIndex idx)
    {
        uint8_t i = static_cast<uint8_t>(idx);
        if (i >= kDmMotorCount)
        {
            return nullptr;
        }
        return &motors_[i];
    }

} // namespace Robot