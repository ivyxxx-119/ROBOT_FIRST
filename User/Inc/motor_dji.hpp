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
 *       覆盖先发的帧。合并逻辑在 RobotController::applyChassisMotors() 中
 *       统一组包，SubTrackMotorController 和 LiftMotorController 只负责
 *       计算各自的目标电流，不独立发送 CAN 帧。
 */

#pragma once

#include <cstdint>
#include <cstring>
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

    // 升降 M2006：与副履带共用 0x1FF 命令帧，独立反馈 ID
    inline constexpr uint32_t kLiftFeedbackId = 0x207U;

    inline constexpr uint32_t kYawCmdId = 0x2FEU;
    inline constexpr uint32_t kYawFeedbackId = 0x209U;
    inline constexpr uint8_t kYawSlot = 0U;

    inline constexpr int16_t kM3508MaxCurrent = 16384;
    inline constexpr int16_t kM2006MaxCurrent = 10000;
    inline constexpr int16_t kGm6020MaxCurrent = 30000;
    inline constexpr int16_t kLiftGravityFF = 500; ///< 升降重力补偿前馈电流，需实测标定（正=抵抗重力方向）

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

    bool DjiSendCanFrame(CAN_HandleTypeDef *hcan,
                         uint32_t cmdId,
                         const int16_t *outputs,
                         uint8_t count);

    static inline int16_t DjiClampOutput(int16_t v, int16_t maxVal)
    {
        if (v > maxVal)
            return maxVal;
        if (v < -maxVal)
            return -maxVal;
        return v;
    }

    /* ======================== M3508 主履带控制器 ======================== */

    class ChassisMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetSpeedRpm(uint8_t idx, float rpm);
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
    // 只计算电流，不独立发送 CAN 帧
    // 外部通过 getTargetOutput() 读取后与升降合并发送

    class SubTrackMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetSpeedRpm(uint8_t idx, float rpm);

        /// 计算PID，更新 targetOutput，不发送CAN
        void calcOutput();

        /// 清零两台电机的 PID 积分器（Standby 时调用，防止积分残留）
        void resetPid();

        int16_t getTargetOutput(uint8_t idx) const;
        const DjiMotorInstance *getInstance(uint8_t idx) const;

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();

    private:
        DjiMotorInstance motors_[kSubTrackMotorCount];
        CAN_HandleTypeDef *hcan_ = nullptr;
    };

    /* ======================== M2006 升降控制器 ======================== */
    /**
     * @brief  夹爪升降电机控制器
     *
     * 位置闭环：
     *   外环：位置误差 → 目标转速
     *   内环：转速误差 → 电流（复用 speedPid）
     *
     * 位置单位：电机编码器累计 count（8192 count/圈）。
     * 上电后以当前位置为零点；建议增加底部回零开关后再换算高度。
     */
    class LiftMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);

        /// 设置目标位置（编码器 count，正方向=上升，需实测确认）
        void setTargetPosition(float posCnt);

        /// 获取当前反馈位置
        float currentPosition() const;

        /// 摇杆输入更新目标（每次调用增减，摇杆回中时不变）
        void updateTargetByJoystick(float normalizedInput, float dtSeconds);

        /// 计算PID，更新 targetOutput，不发送CAN
        void calcOutput();

        int16_t getTargetOutput() const;

        /// 是否接近安全折叠高度
        bool isSafeToFold(float safePos, float thresh) const;

        void updateFeedback(uint32_t canId, const uint8_t *pData);
        void updateOnlineStatus();
        bool isOnline() const;

        /// 同步目标到当前位置（模式切换时防跳变）
        void syncTargetToCurrent();

    private:
        DjiMotorInstance motor_;
        CAN_HandleTypeDef *hcan_ = nullptr;

        float targetPosCnt_ = 0.0f; ///< 目标位置（编码器count）
        float posKp_ = 0.05f;       ///< 位置环比例增益，需实测
        float maxRpm_ = 800.0f;
        float ratePerSec_ = 5000.0f; ///< 摇杆满偏时目标高度变化速率
    };

    /* ======================== GM6020 Yaw控制器 ======================== */

    class YawMotorController
    {
    public:
        void init(CAN_HandleTypeDef *hcan);
        void setTargetAngleDeg(float angleDeg);
        void syncTargetToCurrent();
        float currentAngleDeg() const;
        bool sendCurrent();
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