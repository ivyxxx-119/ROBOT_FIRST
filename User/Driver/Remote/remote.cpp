/**
 * @file    remote.cpp
 * @brief   大疆DR16遥控器DBUS解析实现（C++）
 *
 * DBUS帧格式（18字节，Byte0起）：
 *
 * Byte0～5：
 *   ch0～ch3：四个11-bit摇杆通道
 *   Byte5 bit[5:4]：S2
 *   Byte5 bit[7:6]：S1
 *
 * Byte6～15：鼠标与键盘数据
 * Byte16～17：拨轮 ch4，小端
 *
 * @note 需要 UART 配置为 100000 baud, 8E1（字长9bit+偶校验）。
 *       CubeMX中：WordLength=9Bits, Parity=Even, StopBits=1。
 *       注意：左右摇杆与S1/S2的物理对应关系须实测确认。
 */

#include "remote.hpp"
#include <cstring>

namespace Robot
{

    void RemoteReceiver::init(UART_HandleTypeDef *huart)
    {
        huart_ = huart;
        data_ = RemoteData{};
        data_.switchRight = SwitchPos::Mid;
        data_.switchLeft = SwitchPos::Mid;

        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rxBuf_, sizeof(rxBuf_));
    }

    void RemoteReceiver::parseDbus(const uint8_t *pData, uint16_t len)
    {
        if (pData == nullptr || len != kDbusFrameLen)
        {
            return;
        }

        // 前四个通道：仍可使用 A 的 11-bit 连续位提取函数
        const uint16_t ch0 = extractChannel(pData, 0U);
        const uint16_t ch1 = extractChannel(pData, 1U);
        const uint16_t ch2 = extractChannel(pData, 2U);
        const uint16_t ch3 = extractChannel(pData, 3U);

        // B 的布局：S2 在 Byte5 bit[5:4]，S1 在 bit[7:6]
        const uint8_t rightSwitchRaw =
            static_cast<uint8_t>((pData[5] >> 4U) & 0x03U);

        const uint8_t leftSwitchRaw =
            static_cast<uint8_t>((pData[5] >> 6U) & 0x03U);

        // B 的布局：拨轮在 Byte16～17，小端存储
        const uint16_t dialRaw =
            static_cast<uint16_t>(pData[16]) |
            static_cast<uint16_t>(
                static_cast<uint16_t>(pData[17]) << 8U);

        // 以下范围需要与你们遥控器实际数据核对。
        // 常见中心值为1024；这里使用已有的最大值构造一个对称检查范围。
        const uint16_t channelMin =
            static_cast<uint16_t>(
                2U * kDbusChannelMid - kDbusChannelMax);

        auto validChannel = [channelMin](uint16_t raw)
        {
            return raw >= channelMin && raw <= kDbusChannelMax;
        };

        auto validSwitch = [](uint8_t raw)
        {
            return raw >= 1U && raw <= 3U;
        };

        if (!validChannel(ch0) ||
            !validChannel(ch1) ||
            !validChannel(ch2) ||
            !validChannel(ch3) ||
            !validChannel(dialRaw) ||
            !validSwitch(rightSwitchRaw) ||
            !validSwitch(leftSwitchRaw))
        {
            // 无效帧不能刷新在线时间
            return;
        }

        RemoteData next = data_;

        next.rightH = toNormalized(ch0);
        next.rightV = toNormalized(ch1);

        // B 的代码注释对应：ch2=左水平，ch3=左垂直。
        // 必须在无动力状态下实际拨动摇杆核对。
        next.leftH = toNormalized(ch2);
        next.leftV = toNormalized(ch3);

        next.dial = toNormalized(dialRaw);

        // 以下转换要求 SwitchPos 枚举的数值与遥控器原始值一致。
        // 常见约定是 Up=1、Down=2、Mid=3，须核对 remote.hpp。
        next.switchRight = static_cast<SwitchPos>(rightSwitchRaw);
        next.switchLeft = static_cast<SwitchPos>(leftSwitchRaw);

        next.lastUpdateMs = HAL_GetTick();
        next.online = true;
        data_ = next;
    }

    void RemoteReceiver::updateOnlineStatus()
    {
        if ((HAL_GetTick() - data_.lastUpdateMs) > kRemoteOfflineTimeMs)
        {
            data_.online = false;
        }
    }

    uint16_t RemoteReceiver::extractChannel(const uint8_t *pData,
                                            uint8_t channelIndex)
    {
        // DBUS：每通道11bit，从Byte0 bit0开始连续排列
        uint32_t bitOffset = static_cast<uint32_t>(channelIndex) * 11U;
        uint32_t byteOffset = bitOffset / 8U;
        uint32_t bitShift = bitOffset % 8U;

        uint32_t raw = static_cast<uint32_t>(pData[byteOffset]) | (static_cast<uint32_t>(pData[byteOffset + 1U]) << 8U) | (static_cast<uint32_t>(pData[byteOffset + 2U]) << 16U);

        return static_cast<uint16_t>((raw >> bitShift) & 0x07FFU);
    }

    float RemoteReceiver::toNormalized(uint16_t raw)
    {
        int16_t centered = static_cast<int16_t>(raw) - static_cast<int16_t>(kDbusChannelMid);

        if (centered > -static_cast<int16_t>(kDbusChannelDeadband) &&
            centered < static_cast<int16_t>(kDbusChannelDeadband))
        {
            return 0.0f;
        }

        float n = static_cast<float>(centered) / static_cast<float>(kDbusChannelMax - kDbusChannelMid);

        if (n > 1.0f)
        {
            n = 1.0f;
        }
        if (n < -1.0f)
        {
            n = -1.0f;
        }
        return n;
    }

} // namespace Robot
