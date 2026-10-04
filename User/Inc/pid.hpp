/**
 * @file    pid.hpp
 * @brief   通用PID控制器模块（C++）
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace Robot
{

    enum class PidMode : uint8_t
    {
        Position = 0U,
        Increment = 1U
    };

    struct PidConfig
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float outputLimit = 16384.0f;
        float integralLimit = 5000.0f;
        PidMode mode = PidMode::Position;
    };

    class Pid
    {
    public:
        Pid() = default;
        explicit Pid(const PidConfig &cfg);

        void init(const PidConfig &cfg);
        float update(float setpoint, float measurement);
        void reset();

        float output() const { return output_; }
        float error() const { return error_; }
        bool initialized() const { return initialized_; }
        // 增加调试结构体
        struct DebugData
        {
            volatile float setpoint;
            volatile float measurement;
            volatile float error;
            volatile float proportional;
            volatile float integral;
            volatile float derivative;
            volatile float output;
        };

        DebugData debug_{};

    private:
        PidConfig cfg_{};

        float setpoint_ = 0.0f;
        float measurement_ = 0.0f;
        float error_ = 0.0f;
        float prevError_ = 0.0f;
        float prevPrevError_ = 0.0f;

        float proportional_ = 0.0f;
        float integral_ = 0.0f;
        float derivative_ = 0.0f;
        float output_ = 0.0f;

        bool initialized_ = false;

        static float clamp(float v, float limit);
        float calcPosition(float error);
        float calcIncrement(float error);
    };

} // namespace Robot