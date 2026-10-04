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
    // 副履带与主履带速度比例；因传动比/轮径不同单独标定
    inline constexpr float kSubTrackRatio = 1.95f; ///< 副/主 减速箱输出同速比（实测微调）
    inline constexpr float kSubTrackMaxRpm = 3000.0f;

    // ---------- 升降 M2006 ----------
    // CAN ID：CAN2，ID=0x207，反馈0x207
    // slot在0x1FF帧中占第3位（index=2）
    inline constexpr uint32_t kLiftMotorFeedbackId = 0x207U;
    inline constexpr uint8_t kLiftMotorSlot = 2U; ///< 0x1FF帧的slot索引

    inline constexpr float kLiftTargetRatePerSec = 5000.0f; ///< 目标高度变化速率(编码器count/s)
    inline constexpr float kLiftPosKp = 0.05f;              ///< 位置环Kp，需实测标定
    inline constexpr float kLiftMaxRpm = 800.0f;            ///< 升降最高转速，需实测
    inline constexpr float kLiftSafePosFold = 0.0f;         ///< 允许折叠的安全高度(编码器count)
    inline constexpr float kLiftSafePosThresh = 200.0f;     ///< 安全高度容差

    // ---------- DM4310 关节 ----------
    inline constexpr float kJointFoldDeg = 0.0f;        ///< 收起位，需实测
    inline constexpr float kJointFlatDeg = 0.0f;        ///< 平地收起（同上）
    inline constexpr float kJointMinDeg = -60.0f;       ///< 机械下限，需实测
    inline constexpr float kJointMaxDeg = 60.0f;        ///< 机械上限，需实测
    inline constexpr float kJointRateDegPerSec = 180.0f; ///< 摇杆满偏时关节角速度
    inline constexpr float kJointKp = 10.0f;
    inline constexpr float kJointKd = 1.5f;
    inline constexpr float kJointFoldKp = 10.0f;
    inline constexpr float kJointFoldKd = 1.5f;

    // ---------- Yaw GM6020 ----------
    inline constexpr float kYawMaxDeg = 180.0f;
    inline constexpr float kYawStepDeg = 2.0f; ///< 每次调用的角度步进

    // ---------- 舵机脉宽 (us) ----------
    inline constexpr uint32_t kServoPwmMinUs = 500U;
    inline constexpr uint32_t kServoPwmMaxUs = 2500U;

    inline constexpr uint32_t kFoldFlatUs = 700U;    ///< 收起（水平）
    inline constexpr uint32_t kFoldDeployUs = 2300U; ///< 展开（竖直）

    inline constexpr uint32_t kGripperOpenUs = 2000U;
    inline constexpr uint32_t kGripperCloseUs = 1000U;

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
        Standby = 1U, ///< 右拨杆 Up
        Drive = 2U,   ///< 右拨杆 Mid  平地行驶
        Stair = 3U,   ///< 右拨杆 Down 爬楼梯
        Grab = 4U     ///< 左拨杆 Down 抓取
    };

    enum class GripperState : uint8_t
    {
        Open = 0U,
        Closed = 1U
    };

    /// 升降机构折叠状态
    enum class FoldState : uint8_t
    {
        Folded = 0U,  ///< 收起（水平）
        Deployed = 1U ///< 展开（竖直）
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
        ChassisMotorController chassis_;   ///< M3508 ×2，CAN1
        SubTrackMotorController subTrack_; ///< M2006 ×2，CAN2，副履带
        LiftMotorController lift_;         ///< M2006 ×1，CAN2，升降
        YawMotorController yaw_;           ///< GM6020，CAN1
        DmMotorController joint_;          ///< DM4310 ×2，CAN2
        RemoteReceiver remote_;

        // ---------- 整车状态 ----------
        RobotState state_ = RobotState::Init;
        RobotState prevState_ = RobotState::Init;
        GripperState gripperState_ = GripperState::Open;
        FoldState foldState_ = FoldState::Folded;

        // ---------- 控制目标 ----------
        float targetYawDeg_ = 0.0f;
        float dmTargetDeg_ = kJointFoldDeg; ///< 两侧关节共用目标（可后期分离）

        ChassisCmd chassisCmd_{};
        JointCmd jointCmd_{};
        ServoCmd servoCmd_{};

        bool remoteOnline_ = false;
        bool motorAllOnline_ = false;
        bool firstTask_ = true;
        bool grabRequested_ = false;          ///< 左拨杆下沿触发的 Grab 请求，Up 清除
        SwitchPos prevSwitchLeft_ = SwitchPos::Mid; ///< 上一帧左拨杆位置，用于边沿检测

        // Grab 模式左摇杆轴仲裁结果（只有主导轴非零）
        float grabLeftVArb_ = 0.0f; ///< 仲裁后升降输入
        float grabLeftHArb_ = 0.0f; ///< 仲裁后 Yaw 输入

        // ---------- HAL 句柄 ----------
        CAN_HandleTypeDef *hcan1_ = nullptr;
        CAN_HandleTypeDef *hcan2_ = nullptr;
        TIM_HandleTypeDef *htim1_ = nullptr;
        UART_HandleTypeDef *huart3_ = nullptr;

        // ---------- 私有方法 ----------
        void startCan();
        void updateStateMachine();
        void resolveLeftStickAxis();  ///< Grab模式左摇杆轴仲裁，结果写入 grabLeftVArb_/grabLeftHArb_
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

        void setServoPulse(uint32_t channel, uint32_t pulseUs);
        uint32_t clampPulse(uint32_t v, uint32_t minV, uint32_t maxV);
        float applyDeadband(float v, float db);

        static float clampF(float v, float lo, float hi);
    };

} // namespace Robot

#endif /* __cplusplus */