/**
 * @file    robot.hpp
 * @brief   整车控制主模块 C++接口
 *
 * 硬件结构：
 *   主履带驱动  M3508  ×2  CAN1  0x201~0x202  差速转向
 *   副履带驱动  M2006  ×2  CAN2  0x205~0x206  随主履带同步
 *   副履带关节  DM4310 ×2  CAN2  MIT位置控制
 *   夹爪升降    M2006  ×1  CAN2  0x207        位置闭环
 *   Yaw旋转    GM6020 ×1  CAN1  0x209        位置串级控制
 *   折叠舵机          ×1  PWM TIM1_CH2       升降机构折叠/展开
 *   夹爪舵机          ×1  PWM TIM1_CH3       抓取/释放
 *
 * 状态机：
 *   右拨杆 Up   → Standby
 *   右拨杆 Mid  → Drive
 *   右拨杆 Down → Stair
 *   左拨杆 Down → Grab（优先于 Drive/Stair，不覆盖失联）
 *
 * 遥控通道分配：
 *   rightV / rightH : 主、副履带前进/转向（所有行驶模式）
 *   leftV           : Stair→DM关节角度增减；Grab→升降目标高度增减
 *   leftH           : Grab→Yaw角度增减；其余模式锁定
 *   dial            : Grab→夹爪开合
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
    void Robot_Init(void);
    void Robot_Task(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstdint>
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "remote.hpp"

namespace Robot
{
    /* ======================== 配置常量 ======================== */

    // ---------- 主履带 M3508 ----------
    inline constexpr float kTrackMaxRpm = 4000.0f;
    inline constexpr float kTrackClimbRpm = 2000.0f; ///< Stair/Grab 限速
    inline constexpr float kTrackGrabRpm = 1500.0f;  ///< Grab 低速对位

    // ---------- 副履带 M2006（跟随主履带）----------
    inline constexpr float kSubTrackRatio = 1.95f; ///< 副/主 减速箱输出同速比（实测微调）
    inline constexpr float kSubTrackMaxRpm = 3000.0f;

    // ---------- 升降 M2006 ----------
    inline constexpr uint32_t kLiftMotorFeedbackId = 0x207U;
    inline constexpr uint8_t kLiftMotorSlot = 2U;
    inline constexpr float kLiftTargetRatePerSec = 5000.0f;
    inline constexpr float kLiftPosKp = 0.05f;
    inline constexpr float kLiftMaxRpm = 800.0f;
    inline constexpr float kLiftSafePosFold = 0.0f;
    inline constexpr float kLiftSafePosThresh = 200.0f;

    // ---------- DM4310 关节 ----------
    inline constexpr float kJointFoldDeg = 0.0f;
    inline constexpr float kJointFlatDeg = 0.0f;
    inline constexpr float kJointMinDeg = -60.0f;
    inline constexpr float kJointMaxDeg = 60.0f;
    inline constexpr float kJointRateDegPerSec = 180.0f;
    inline constexpr float kJointKp = 10.0f;
    inline constexpr float kJointKd = 1.5f;
    inline constexpr float kJointFoldKp = 10.0f;
    inline constexpr float kJointFoldKd = 1.5f;

    // ---------- Yaw GM6020 ----------
    inline constexpr float kYawMaxDeg = 180.0f;
    inline constexpr float kYawStepDeg = 2.0f;

    // ---------- 舵机脉宽 (us) ----------
    inline constexpr uint32_t kServoPwmMinUs = 500U;
    inline constexpr uint32_t kServoPwmMaxUs = 2500U;
    inline constexpr uint32_t kFoldFlatUs = 2170U;   // 往大是往下
    inline constexpr uint32_t kFoldDeployUs = 1240U; // 往小是往上
    inline constexpr uint32_t kGripperOpenUs = 1200U;
    inline constexpr uint32_t kGripperCloseUs = 1200U;

    // ---------- 舵机 PWM 通道（TIM1）----------
    inline constexpr uint32_t kServoFoldCh = TIM_CHANNEL_2;    ///< TIM1_CH2 折叠舵机
    inline constexpr uint32_t kServoGripperCh = TIM_CHANNEL_3; ///< TIM1_CH3 夹爪舵机
    // ---------- 遥控死区 ----------
    inline constexpr float kJoystickDeadband = 0.05f;
    inline constexpr float kDialGripperThresh = 0.5f;

    // ---------- 转向比例 ----------
    inline constexpr float kTurnScale = 0.6f;

    // ---------- 任务周期 ----------
    inline constexpr float kTaskPeriodS = 0.002f; ///< 2 ms

    /* ======================== 枚举 ======================== */

    enum class RobotState : uint8_t
    {
        Init = 0U,
        Standby = 1U,
        Drive = 2U,
        Stair = 3U,
        Grab = 4U
    };

    enum class GripperState : uint8_t
    {
        Open = 0U,
        Closed = 1U
    };

    enum class FoldState : uint8_t
    {
        Folded = 0U,
        Deployed = 1U
    };

    /* ======================== 指令结构 ======================== */

    struct ChassisCmd
    {
        float leftRpm = 0.0f;
        float rightRpm = 0.0f;
    };

    struct JointCmd
    {
        float leftDeg = 0.0f;
        float rightDeg = 0.0f;
        float velDegPerS = 0.0f;
        float kp = kJointKp;
        float kd = kJointKd;
    };

    struct ServoCmd
    {
        uint32_t foldUs = kFoldFlatUs;
        uint32_t gripperUs = kGripperOpenUs;
    };

    /* ======================== 整车控制器 ======================== */

    class RobotController
    {
    public:
        RobotController() = default;

        void init();
        void task();

        void can1RxDispatch(uint32_t canId, const uint8_t *pData);
        void can2RxDispatch(uint32_t canId, const uint8_t *pData, uint8_t dlc);
        void onUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);

        RobotState state() const { return state_; }
        bool isRemoteOnline() const { return remote_.data().online; }

    private:
        // ---------- 电机/外设控制器 ----------
        ChassisMotorController chassis_;
        SubTrackMotorController subTrack_;
        LiftMotorController lift_;
        YawMotorController yaw_;
        DmMotorController joint_;
        RemoteReceiver remote_;

        // ---------- 整车状态 ----------
        RobotState state_ = RobotState::Init;
        RobotState prevState_ = RobotState::Init;
        GripperState gripperState_ = GripperState::Open;
        FoldState foldState_ = FoldState::Folded;

        // ---------- 控制目标 ----------
        float targetYawDeg_ = 0.0f;
        float dmTargetDeg_ = kJointFoldDeg;

        ChassisCmd chassisCmd_{};
        JointCmd jointCmd_{};
        ServoCmd servoCmd_{};

        bool remoteOnline_ = false;
        bool motorAllOnline_ = false;
        bool firstTask_ = true;
        bool grabRequested_ = false;
        SwitchPos prevSwitchLeft_ = SwitchPos::Mid;

        float grabLeftVArb_ = 0.0f;
        float grabLeftHArb_ = 0.0f;

        // ---------- HAL 句柄 ----------
        CAN_HandleTypeDef *hcan1_ = nullptr;
        CAN_HandleTypeDef *hcan2_ = nullptr;
        TIM_HandleTypeDef *htim1_ = nullptr;
        UART_HandleTypeDef *huart3_ = nullptr;

        // ---------- 私有方法 ----------
        void startCan();
        void updateStateMachine();
        void resolveLeftStickAxis();
        void resolveChassisCmd();
        void resolveJointCmd();
        void resolveYawCmd();
        void resolveServoCmd();
        void resolveLiftCmd();
        void applyChassisMotors();
        void applyJointMotors();
        void applyYawMotor();
        void applyServos();
        void applyLiftMotor();
        void checkOnlineStatus();
        void faultStop();
        void onStateEnter(RobotState newState);
        bool sendMotorCommands();
        void setServoPulse(uint32_t channel, uint32_t pulseUs);
        uint32_t clampPulse(uint32_t v, uint32_t minV, uint32_t maxV);
        float applyDeadband(float v, float db);
        static float clampF(float v, float lo, float hi);
    };

} // namespace Robot

#endif /* __cplusplus */