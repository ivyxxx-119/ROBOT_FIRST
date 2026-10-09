/**
 * @file    motor_dji.hpp
 * @brief   大疆电机驱动模块（M3508 / M2006 / GM6020）C++接口
 *
 * CAN ID 分配：
 *   M3508  主履带 ×2  CAN1  反馈 0x201~0x202  命令 0x200
 *   M2006  副履带 ×2  CAN2  反馈 0x205~0x206  命令 0x1FF slot0~1
 *   M2006  升降   ×1  CAN2  反馈 0x207        命令 0x1FF slot2
 *   GM6020 Yaw   ×1  CAN1  反馈 0x209        命令 0x2FE slot0
 *
 * 注意：0x1FF 帧共4个slot（slot0~3），副履带占slot0~1，升降占slot2。
 *       三台 M2006 的命令必须合并在同一个 0x1FF 帧发出，否则后发的帧会
 *       覆盖先发的帧。合并逻辑在 RobotController::sendMotorCommands() 中
 *       统一组帧，SubTrackMotorController 和 LiftMotorController 只负责
 *       计算各自的目标电流，不独立发送 CAN 帧。
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include "pid.hpp"
#include "stm32f4xx_hal.h"

namespace Robot
{
    /* ======================== 常量 ======================== */

    inline constexpr uint8_t kChassisMotorCount = 2U;
    inline constexpr uint32_t kChassisCmdId = 0x200U;
    inline constexpr uint32_t kChassisFeedbackBase = 0x201U;

    inline constexpr uint8_t kSubTrackMotorCount = 2U;
    inline constexpr uint32_t kSubTrackCmdId = 0x1FFU;
    inline constexpr uint32_t kSubTrackFeedbackBase = 0x205U;

    inline constexpr uint32_t kLiftFeedbackId = 0x207U;

    inline constexpr uint32_t kYawCmdId = 0x2FEU;
    inline constexpr uint32_t kYawFeedbackId = 0x209U;
    inline constexpr uint8_t kYawSlot = 0U;

    inline constexpr int16_t kM3508MaxCurrent = 16384;
    inline constexpr int16_t kM2006MaxCurrent = 10000;
    inline constexpr int16_t kGm6020MaxCurrent = 30000;
    inline constexpr int16_t kLiftGravityFF = 500;

    inline constexpr uint32_t kDjiOfflineTimeoutMs = 100U;

    /* ======================== 反馈结构 ======================== */

    struct DjiMotorFeedback
    {
        uint16_t angleRaw = 0U;
        int16_t speedRpm = 0;
        int16_t torqueCurrent = 0;
        uint8_t temperature = 0U;
    };

    /* ======================== 电机类型 ======================== */

    enum class DjiMotorType : uint8_t
    {
        M3508 = 0U,
        M2006 = 1U,
        Gm6020 = 2U
    };

    /* ======================== 单电机实例 ======================== */

    class DjiMotorInstance
    {
    public:
        DjiMotorInstance() = default;

        void init(uint32_t feedbackId,
                  uint32_t commandId,
                  uint8_t slot,
                  DjiMotorType type,
                  const PidConfig &speedCfg,
                  const PidConfig *posCfg = nullptr);

        void updateFeedback(const uint8_t *pData);
        void updateOnlineStatus(uint32_t nowMs);
        void resetAngle();

        float angleDeg() const { return angleDeg_; }
        const DjiMotorFeedback &feedback() const { return fb_; }
        bool isOnline() const { return online_; }
        uint32_t feedbackId() const { return feedbackId_; }
        uint8_t slot() const { return slot_; }
        DjiMotorType type() const { return type_; }

        Pid speedPid;
        Pid posPid;

        float targetSpeedRpm = 0.0f;
        float targetAngleDeg = 0.0f;
        int16_t targetOutput = 0;

    private:
        uint32_t feedbackId_ = 0U;
        uint32_t commandId_ = 0U;
        uint8_t slot_ = 0U;
        DjiMotorType type_ = DjiMotorType::M3508;
        DjiMotorFeedback fb_{};

        uint16_t lastEncodeRaw_ = 0U;
        int32_t roundCount_ = 0;
        uint16_t encodeOffset_ = 0U;
        float angleDeg_ = 0.0f;
        bool angleInited_ = false;
        bool online_ = false;
        uint32_t lastUpdateMs_ = 0U;
    };

    /* ======================== CAN帧工具 ======================== */

    /**
     * @brief 将四路 int16_t 输出编码为 DJI 8字节协议帧并发送。
     *        底层调用 BspCanSendStdFrame()。
     */
    bool DjiSendCanFrame(CAN_HandleTypeDef *hcan,
                         uint32_t cmdId,
                         const int16_t *outputs,
                         uint8_t count);

    /**
     * @brief 整数域限幅（先转换再限幅，仅用于已知安全的整数值）。
     */
    static inline int16_t DjiClampOutput(int16_t v, int16_t maxVal)
    {
        if (v > maxVal)
            return maxVal;
        if (v < -maxVal)
            return -maxVal;
        return v;
    }

    /**
     * @brief 浮点域检查、限幅，再转换为电机控制整数。
     *        在浮点限幅后再转换，避免溢出或 NaN 转整数的未定义行为。
     */
    static inline int16_t DjiOutputFromFloat(float value, int16_t maxVal)
    {
        if (!std::isfinite(value) || maxVal <= 0)
        {
            return 0;
        }
        const float limit = static_cast<float>(maxVal);
        if (value > limit)
            value = limit;
        if (value < -limit)
            value = -limit;
        return static_cast<int16_t>(value);
    }

    /* ======================== M3508 主履带控制器 ======================== */

    class ChassisMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetSpeedRpm(uint8_t idx, float rpm);

        /// 只计算控制输出，不发送 CAN
        void calcOutput();

        /// 读取某台电机计算好的输出
        int16_t getTargetOutput(uint8_t idx) const;

        /// 清零目标、输出并重置 PID，不发送 CAN
        void clearOutput();

        /// 兼容接口：内部调用 calcOutput() 后发送，迁移完成后删除
        bool sendAllCurrent();

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();
        bool isAllOnline() const;
        const DjiMotorInstance *getInstance(uint8_t idx) const;

    private:
        DjiMotorInstance motors_[kChassisMotorCount];
        CAN_HandleTypeDef *hcan_ = nullptr;
    };

    /* ======================== M2006 副履带控制器 ======================== */

    class SubTrackMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetSpeedRpm(uint8_t idx, float rpm);

        /// 计算PID，更新 targetOutput，不发送CAN
        void calcOutput();

        /// 清零 PID 积分器、目标和输出
        void resetPid();

        /// 清零目标、输出并重置 PID，不发送 CAN
        void clearOutput();

        int16_t getTargetOutput(uint8_t idx) const;
        const DjiMotorInstance *getInstance(uint8_t idx) const;

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();

    private:
        DjiMotorInstance motors_[kSubTrackMotorCount];
        CAN_HandleTypeDef *hcan_ = nullptr;
    };

    /* ======================== M2006 升降控制器 ======================== */

    class LiftMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);

        void setTargetPosition(float posCnt);
        float currentPosition() const;
        void updateTargetByJoystick(float normalizedInput, float dtSeconds);

        /// 计算PID，更新 targetOutput，不发送CAN
        void calcOutput();

        int16_t getTargetOutput() const;
        bool isSafeToFold(float safePos, float thresh) const;

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();
        bool isOnline() const;

        void syncTargetToCurrent();

    private:
        DjiMotorInstance motor_;
        CAN_HandleTypeDef *hcan_ = nullptr;

        float targetPosCnt_ = 0.0f;
        float posKp_ = 0.05f;
        float maxRpm_ = 800.0f;
        float ratePerSec_ = 5000.0f;
    };

    /* ======================== GM6020 Yaw控制器 ======================== */

    class YawMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetAngleDeg(float angleDeg);
        void syncTargetToCurrent();
        float currentAngleDeg() const;

        /// 只计算串级 PID，不发送 CAN
        void calcOutput();

        /// 读取计算好的控制输出
        int16_t getTargetOutput() const;

        /// 清零输出并重置 PID，不发送 CAN
        void clearOutput();

        /// 兼容接口：内部调用 calcOutput() 后发送，迁移完成后删除
        bool sendCurrent();

        /// 兼容接口：内部调用 clearOutput() 后发送，迁移完成后删除
        bool sendZeroCurrent();

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();
        bool isOnline() const;
        const DjiMotorInstance *getInstance() const { return &motor_; }

    private:
        DjiMotorInstance motor_;
        CAN_HandleTypeDef *hcan_ = nullptr;
        float targetAngleDeg_ = 0.0f;

        void runCascadePid();
    };

} // namespace Robot