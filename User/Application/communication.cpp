/**
 * @file    communication.cpp
 * @brief   CAN/UART 接收分发 + 所有电机命令统一发送
 */

#include "robot.hpp"
#include "main.h"
#include "debug.hpp"
#include "bsp_can.hpp"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
}

namespace Robot
{

    /* ----------------------------------------------------------------
     * CAN1 接收分发
     * ---------------------------------------------------------------- */
    void RobotController::can1RxDispatch(uint32_t canId,
                                         const uint8_t *pData)
    {
        if (pData == nullptr)
            return;

        if (canId == 0x201U)
        {
            ++debug201RxCount;
            debug201SpeedRpm = static_cast<int16_t>(
                (static_cast<uint16_t>(pData[2]) << 8U) | pData[3]);
        }
        else if (canId == 0x202U)
        {
            ++debug202RxCount;
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

    /* ----------------------------------------------------------------
     * CAN2 接收分发
     * ---------------------------------------------------------------- */
    void RobotController::can2RxDispatch(uint32_t canId,
                                         const uint8_t *pData,
                                         uint8_t dlc)
    {
        if (pData == nullptr)
            return;

        debugCan2LastRxId = canId;
        ++debugCan2RxCount;

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

        if (canId >= kSubTrackFeedbackBase &&
            canId < kSubTrackFeedbackBase + kSubTrackMotorCount)
        {
            subTrack_.updateFeedback(canId, pData);
            return;
        }

        if (canId == kLiftFeedbackId)
        {
            lift_.updateFeedback(canId, pData);
            return;
        }

        if (canId == kDmJointLeftRxCanId)
            ++debugDmRxLeftCount;
        else if (canId == kDmJointRightRxCanId)
            ++debugDmRxRightCount;

        joint_.updateFeedback(canId, pData, dlc);
    }

    /* ----------------------------------------------------------------
     * UART 接收事件
     * ---------------------------------------------------------------- */
    void RobotController::onUartRxEvent(UART_HandleTypeDef *huart,
                                        uint16_t size)
    {
        if (huart == nullptr || huart->Instance != huart3_->Instance)
            return;

        ++debugUartRxEventCount;
        debugUartRxSize = size;

        if (size == kDbusFrameLen)
        {
            const uint32_t previousMs = remote_.data().lastUpdateMs;
            remote_.parseDbus(remote_.rxBuf(), size);

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
     * startCan
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

    /* ================================================================
     * sendMotorCommands
     *
     * 调用前提：当前周期内各 apply*() 已执行完毕，
     *           targetOutput 均已更新。
     * 每个控制周期由 task() 调用一次；faultStop() 也调用一次。
     * 此函数内不运行任何 PID，不修改控制目标。
     * ================================================================ */
    bool RobotController::sendMotorCommands()
    {
        bool allOk = true;

        /* ---- 1. 主履带 M3508：CAN1  0x200 ---- */
        {
            int16_t frame[4] = {};
            frame[0] = chassis_.getTargetOutput(0U);
            frame[1] = chassis_.getTargetOutput(1U);

            debugChTx201 = frame[0];
            debugChTx202 = frame[1];

            allOk = DjiSendCanFrame(hcan1_, kChassisCmdId, frame, 4U) && allOk;
        }

        /* ---- 2. 副履带 + 升降 M2006：CAN2  0x1FF ---- */
        {
            int16_t frame[4] = {};
            frame[0] = subTrack_.getTargetOutput(0U);
            frame[1] = subTrack_.getTargetOutput(1U);
            frame[2] = lift_.getTargetOutput();

            debugOut205 = frame[0];
            debugOut206 = frame[1];
            debugOut207 = frame[2];

            allOk = DjiSendCanFrame(hcan2_, kSubTrackCmdId, frame, 4U) && allOk;
        }

        /* ---- 3. Yaw GM6020：CAN1  0x2FE ---- */
        {
            static_assert(kYawSlot < 4U, "kYawSlot out of range");

            int16_t frame[4] = {};
            frame[kYawSlot] = yaw_.getTargetOutput();

            allOk = DjiSendCanFrame(hcan1_, kYawCmdId, frame, 4U) && allOk;
        }

        /* ---- 4. DM4310 关节：CAN2，左右各一帧 ---- */
        {
            const DmIndex indices[kDmMotorCount] = {
                DmIndex::Left,
                DmIndex::Right};

            for (uint8_t i = 0U; i < kDmMotorCount; ++i)
            {
                DmMitFrame frame{};

                if (!joint_.buildMitFrame(indices[i], frame) ||
                    !frame.valid)
                {
                    allOk = false;
                    continue;
                }

                allOk = BspCanSendStdFrame(
                            hcan2_, frame.canId, frame.data, 8U) &&
                        allOk;
            }
        }

        return allOk;
    }

} // namespace Robot