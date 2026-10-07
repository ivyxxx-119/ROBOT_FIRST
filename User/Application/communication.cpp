#include "communication.hpp"

#include "main.h"
#include "debug.hpp"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
}

namespace Robot
{

void RobotController::can1RxDispatch(uint32_t canId, const uint8_t *pData)
    {
        if (canId == 0x201U)
        {
            debug201RxCount++;
            debug201SpeedRpm = static_cast<int16_t>(
                (static_cast<uint16_t>(pData[2]) << 8U) | pData[3]);
        }
        else if (canId == 0x202U)
        {
            debug202RxCount++;
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

void RobotController::can2RxDispatch(uint32_t canId,
                                         const uint8_t *pData,
                                         uint8_t dlc)
    {
        // 调试变量
        debugCan2LastRxId = canId;
        debugCan2RxCount++;

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

        // 副履带 M2006：0x205~0x206
        if (canId >= kSubTrackFeedbackBase &&
            canId < kSubTrackFeedbackBase + kSubTrackMotorCount)
        {
            subTrack_.updateFeedback(canId, pData);
            return;
        }

        // 升降 M2006：0x207
        if (canId == kLiftFeedbackId)
        {
            lift_.updateFeedback(canId, pData);
            return;
        }

        // DM4310 关节
        if (canId == kDmJointLeftRxCanId)
            ++debugDmRxLeftCount;
        else if (canId == kDmJointRightRxCanId)
            ++debugDmRxRightCount;
        joint_.updateFeedback(canId, pData, dlc);
    }

void RobotController::onUartRxEvent(
        UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart == nullptr || huart->Instance != huart3_->Instance)
            return;

        ++debugUartRxEventCount;
        debugUartRxSize = size;

        if (size == kDbusFrameLen)
        {
            const uint32_t previousMs = remote_.data().lastUpdateMs;
            remote_.parseDbus(remote_.rxBuf(), size);

            // 初步判断是否通过解析；更精确的成功计数建议放进 parseDbus()
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

        // CAN1/CAN2 同一物理总线 → 统一用 FIFO0
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

} // namespace Robot
