/**
 * @file    remote.hpp
 * @brief   大疆DR16遥控器DBUS解析模块 C++接口
 *
 * 本模块仅使用 CH0~CH4 和 S1/S2。
 *
 * 通道赋值（与遥控器拨杆对应关系由用户硬件确认）：
 *   CH0 右摇杆水平   CH1 右摇杆垂直
 *   CH2 左摇杆垂直   CH3 左摇杆水平
 *   CH4 拨轮
 */

#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

namespace Robot
{

    /* ======================== 常量 ======================== */

    inline constexpr uint16_t kDbusFrameLen = 18U;
    inline constexpr uint16_t kDbusChannelMid = 1024U;
    inline constexpr uint16_t kDbusChannelMax = 1684U;
    inline constexpr uint16_t kDbusChannelMin = 364U;
    inline constexpr uint16_t kDbusChannelDeadband = 10U;
    inline constexpr uint32_t kRemoteOfflineTimeMs = 300U;

    /* ======================== 开关位置枚举 ======================== */

    enum class SwitchPos : uint8_t
    {
        Up = 1U,
        Mid = 3U,
        Down = 2U
    };

    /* ======================== 遥控器数据 ======================== */

    struct RemoteData
    {
        float rightH = 0.0f; ///< 右摇杆水平 [-1, 1]
        float rightV = 0.0f; ///< 右摇杆垂直 [-1, 1]
        float leftV = 0.0f;  ///< 左摇杆垂直 [-1, 1]
        float leftH = 0.0f;  ///< 左摇杆水平 [-1, 1]
        float dial = 0.0f;   ///< 拨轮       [-1, 1]

        SwitchPos switchRight = SwitchPos::Mid;
        SwitchPos switchLeft = SwitchPos::Mid;

        bool online = false;
        uint32_t lastUpdateMs = 0U;
    };

    /* ======================== 遥控器模块 ======================== */

    class RemoteReceiver
    {
    public:
        void init(UART_HandleTypeDef *huart);

        /**
         * @brief  解析一帧18字节DBUS数据
         * @note   由HAL_UARTEx_RxEventCallback调用
         */
        void parseDbus(const uint8_t *pData, uint16_t len);

        void updateOnlineStatus();

        const RemoteData &data() const { return data_; }

        uint8_t *rxBuf() { return rxBuf_; }
        uint16_t rxBufSize() const
        {
            return static_cast<uint16_t>(sizeof(rxBuf_));
        }

        UART_HandleTypeDef *huart() { return huart_; }

    private:
        UART_HandleTypeDef *huart_ = nullptr;
        uint8_t rxBuf_[kDbusFrameLen * 2U]{};
        RemoteData data_{};

        static uint16_t extractChannel(const uint8_t *pData,
                                       uint8_t channelIndex);
        static float toNormalized(uint16_t raw);
    };

} // namespace Robot