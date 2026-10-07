/**
 * @file    motor_dm.hpp
 * @brief   达妙电机驱动模块（DM4310 MIT模式）C++接口
 *
 * MIT帧格式：pos(16)|vel(12)|kp(12)|kd(12)|tor(12)
 * 使用CAN2，每台电机独立发送
 */

#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

namespace Robot
{

    /* ======================== 常量 ======================== */

    inline constexpr uint8_t kDmMotorCount = 2U;
    inline constexpr uint8_t kDmEnableCmd = 0xFCU;
    inline constexpr uint8_t kDmDisableCmd = 0xFDU;
    inline constexpr uint8_t kDmClearFaultCmd = 0xFBU;

    inline constexpr float kDm4310PosMaxRad = 12.5f;
    inline constexpr float kDm4310VelMaxRadPerS = 30.0f;
    inline constexpr float kDm4310TorqueMax = 10.0f;
    // 转换系数
    inline constexpr float kDmRadToDeg = 57.2957795131f;
    inline constexpr float kDmDegToRad = 0.0174532925199f;

    inline constexpr float kDm4310KpMax = 500.0f;
    inline constexpr float kDm4310KdMax = 5.0f;

    inline constexpr uint32_t kDmOfflineTimeoutMs = 200U;

    inline constexpr uint32_t kDmJointLeftTxCanId = 0x001U;
    inline constexpr uint32_t kDmJointLeftRxCanId = 0x010U;
    inline constexpr uint32_t kDmJointRightTxCanId = 0x002U;
    inline constexpr uint32_t kDmJointRightRxCanId = 0x020U;

    /* ======================== 索引枚举 ======================== */

    enum class DmIndex : uint8_t
    {
        Left = 0U,
        Right = 1U
    };

    /* ======================== 反馈结构 ======================== */

    struct DmMotorFeedback
    {
        uint8_t motorId = 0U;
        uint8_t errorCode = 0U;
        float angleDeg = 0.0f;
        float velocityDegPerS = 0.0f;
        float torque = 0.0f;
    };

    /* ======================== 单电机实例 ======================== */

    class DmMotorInstance
    {
    public:
        DmMotorInstance() = default;
        void init(uint32_t txCanId, uint32_t rxCanId, float kp, float kd);

        // MIT目标
        float targetAngleDeg = 0.0f;
        float targetVelDegPerS = 0.0f;
        float targetTorque = 0.0f;
        float kp = 10.0f;
        float kd = 1.0f;

        uint32_t txCanId() const { return txCanId_; }
        uint32_t rxCanId() const { return rxCanId_; }
        bool isEnabled() const { return enabled_; }
        bool isOnline() const { return online_; }
        const DmMotorFeedback &feedback() const { return fb_; }

        void setEnabled(bool v) { enabled_ = v; }
        void setOnline(bool v) { online_ = v; }
        void updateLastMs(uint32_t ms) { lastUpdateMs_ = ms; }
        void updateOnlineStatus(uint32_t nowMs);
        void parseFeedback(const uint8_t *pData);

    private:
        uint32_t txCanId_ = 0U;
        uint32_t rxCanId_ = 0U;
        bool enabled_ = false;
        bool online_ = false;
        uint32_t lastUpdateMs_ = 0U;
        DmMotorFeedback fb_{};

        static float uintToFloat(uint32_t raw,
                                 float minVal, float maxVal,
                                 uint8_t bits);
    };

    /* ======================== DM控制器 ======================== */

    class DmMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);

        void enable(DmIndex idx);
        void disable(DmIndex idx);
        void clearFault(DmIndex idx);

        void setMitTarget(DmIndex idx,
                          float targetAngleDeg,
                          float targetVelDegPerS,
                          float targetTorque,
                          float kp,
                          float kd);

        bool sendMitCommand(DmIndex idx);

        void updateFeedback(uint32_t canId, const uint8_t *pData, uint8_t dlc);
        void updateOnlineStatus();

        const DmMotorInstance *getInstance(DmIndex idx) const;

    private:
        DmMotorInstance motors_[kDmMotorCount];
        CAN_HandleTypeDef *hcan_ = nullptr;

        static uint32_t floatToUint(float v,
                                    float vMin, float vMax,
                                    uint8_t bits);

        bool sendRawFrame(uint32_t canId,
                          const uint8_t *pData,
                          uint8_t len);

        bool sendSpecialCmd(uint8_t motorCanId, uint8_t cmd);

        DmMotorInstance *getMotor(DmIndex idx);
    };

} // namespace Robot
