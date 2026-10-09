// User/Bsp/bsp_can.hpp
#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>

// 全局命名空间，不加 namespace Robot
bool BspCanSendStdFrame(CAN_HandleTypeDef *hcan,
                        uint32_t stdId,
                        const uint8_t *pData,
                        uint8_t len);