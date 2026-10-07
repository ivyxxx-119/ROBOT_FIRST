#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include "robot.hpp"
#include "main.h"

namespace Robot
{

/*
 * 初始化和舵机输出共用的通道定义。
 * 配置直接归入机械臂模块，不建立robot_config。
 */
static constexpr uint32_t kServoFoldCh =
    TIM_CHANNEL_2;

static constexpr uint32_t kServoGripperCh =
    TIM_CHANNEL_3;

} // namespace Robot

#endif
