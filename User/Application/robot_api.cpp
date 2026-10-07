#include "robot_api.hpp"

#include "robot.hpp"
#include "main.h"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
}

namespace
{

/* 构造阶段只能设置初值，不能访问硬件。 */
Robot::RobotController g_robot;

} // namespace

extern "C" void Robot_Init(void)
{
    g_robot.init();
}

extern "C" void Robot_Task(void)
{
    g_robot.task();
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8] = {0U};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    if (hcan->Instance == hcan1.Instance)
        g_robot.can1RxDispatch(rxHeader.StdId, rxData);
    else if (hcan->Instance == hcan2.Instance)
        g_robot.can2RxDispatch(rxHeader.StdId, rxData, rxHeader.DLC);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8] = {0U};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK)
        return;

    // FIFO1：CAN2（M2006/DM4310）
    if (hcan->Instance == hcan2.Instance)
        g_robot.can2RxDispatch(rxHeader.StdId, rxData, rxHeader.DLC);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    g_robot.onUartRxEvent(huart, size);
}
