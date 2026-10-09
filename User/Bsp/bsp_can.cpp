/**
 * @file    bsp_can.cpp
 * @brief   CAN 硬件发送底层接口实现
 */

#include "bsp_can.hpp"
#include <cstring>

// 注意：不放在 namespace Robot 里，与头文件声明保持一致

bool BspCanSendStdFrame(CAN_HandleTypeDef *hcan,
                        uint32_t canId,
                        const uint8_t *data,
                        uint8_t len)
{
    if (hcan == nullptr ||
        data == nullptr ||
        canId > 0x7FFU ||
        len == 0U ||
        len > 8U)
    {
        return false;
    }

    // 等待空闲邮箱，超时 1 ms 放弃
    const uint32_t startMs = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if ((HAL_GetTick() - startMs) >= 1U)
            return false;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = canId;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;

    uint8_t txBuf[8] = {};
    std::memcpy(txBuf, data, len);

    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(hcan, &header, txBuf, &mailbox) == HAL_OK;
}