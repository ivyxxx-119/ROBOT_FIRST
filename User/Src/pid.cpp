/**
 * @file    pid.cpp
 * @brief   通用PID控制器模块实现（C++）
 */

#include "pid.hpp"

namespace Robot
{

    Pid::Pid(const PidConfig &cfg)
    {
        init(cfg);
    }

    void Pid::init(const PidConfig &cfg)
    {
        cfg_ = cfg;
        initialized_ = true;
        reset();
    }

    float Pid::update(float setpoint, float measurement)
    {
        if (!initialized_)
        {
            return 0.0f;
        }

        setpoint_ = setpoint;
        measurement_ = measurement;
        float error = setpoint - measurement;
        // 增加调试结构体数据
        debug_.setpoint = setpoint;
        debug_.measurement = measurement;
        debug_.error = error;

        if (cfg_.mode == PidMode::Position)
        {
            output_ = calcPosition(error);
        }
        else
        {
            output_ = calcIncrement(error);
        }
        // 增加调试结构体数据
        debug_.proportional = proportional_;
        debug_.integral = integral_;
        debug_.derivative = derivative_;
        debug_.output = output_;

        prevPrevError_ = prevError_;
        prevError_ = error;
        error_ = error;

        return output_;
    }

    void Pid::reset()
    {
        error_ = 0.0f;
        prevError_ = 0.0f;
        prevPrevError_ = 0.0f;
        integral_ = 0.0f;
        proportional_ = 0.0f;
        derivative_ = 0.0f;
        output_ = 0.0f;
    }

    float Pid::clamp(float v, float limit)
    {
        if (limit <= 0.0f)
        {
            return v;
        }
        if (v > limit)
        {
            return limit;
        }
        if (v < -limit)
        {
            return -limit;
        }
        return v;
    }

    float Pid::calcPosition(float error)
    {
        proportional_ = cfg_.kp * error;

        integral_ += cfg_.ki * error;
        integral_ = clamp(integral_, cfg_.integralLimit);

        derivative_ = cfg_.kd * (error - prevError_);
        output_ = clamp(proportional_ + integral_ + derivative_,
                        cfg_.outputLimit);

        return output_;
    }

    float Pid::calcIncrement(float error)
    {
        float inc = cfg_.kp * (error - prevError_) + cfg_.ki * error + cfg_.kd * (error - 2.0f * prevError_ + prevPrevError_);

        output_ += inc;
        return clamp(output_, cfg_.outputLimit);
    }

} // namespace Robot