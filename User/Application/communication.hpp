/**
 * @file    communication.hpp
 * @brief   通信模块头文件
 *
 * RobotController 通信相关成员函数的实现均在 communication.cpp 中：
 *   - can1RxDispatch()     CAN1 反馈分发
 *   - can2RxDispatch()     CAN2 反馈分发
 *   - onUartRxEvent()      遥控串口处理
 *   - startCan()           CAN 启动与过滤器配置
 *   - sendMotorCommands()  统一组帧发送（每周期调用一次）
 *
 * 成员函数声明统一保留在 robot.hpp 的 RobotController 类定义中。
 */

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include "robot.hpp"

#endif