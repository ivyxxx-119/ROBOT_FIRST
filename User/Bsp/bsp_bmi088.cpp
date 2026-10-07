#include "bsp_bmi088.hpp"
#include "bsp_delay.hpp"

#include "main.h"
#include "spi.h"

namespace
{
    constexpr uint16_t kMaxTransferSize = 16U;
    constexpr uint32_t kSpiTimeoutMs = 10U;

    volatile HAL_StatusTypeDef s_last_status = HAL_OK;
    volatile uint32_t s_error_count = 0U;

    bool s_ready = false;

    void release_all()
    {
        HAL_GPIO_WritePin(
            CS1_ACCEL_GPIO_Port,
            CS1_ACCEL_Pin,
            GPIO_PIN_SET);

        HAL_GPIO_WritePin(
            CS1_GYRO_GPIO_Port,
            CS1_GYRO_Pin,
            GPIO_PIN_SET);
    }

    void select_target(BspBmi088Target target)
    {
        if (target == BSP_BMI088_ACCEL)
        {
            HAL_GPIO_WritePin(
                CS1_ACCEL_GPIO_Port,
                CS1_ACCEL_Pin,
                GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(
                CS1_GYRO_GPIO_Port,
                CS1_GYRO_Pin,
                GPIO_PIN_RESET);
        }
    }
}

extern "C" bool BspBmi088_Init(void)
{
    s_ready = false;
    release_all();

    /*
     * GPIO and SPI1 must already have been initialized by CubeMX.
     * This function does not configure those peripherals.
     */
    s_ready = delay_is_ready();
    return s_ready;
}

extern "C" bool BspBmi088_Transfer(
    BspBmi088Target target,
    const uint8_t *tx,
    uint8_t *rx,
    uint16_t length)
{
    if (!s_ready ||
        tx == nullptr ||
        rx == nullptr ||
        length == 0U ||
        length > kMaxTransferSize ||
        (target != BSP_BMI088_ACCEL &&
         target != BSP_BMI088_GYRO))
    {
        return false;
    }

    /*
     * HAL takes a mutable TX pointer.
     * Copy instead of casting away const.
     */
    uint8_t tx_buffer[kMaxTransferSize] = {};

    for (uint16_t i = 0U; i < length; ++i)
    {
        tx_buffer[i] = tx[i];
    }

    release_all();
    select_target(target);

    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(
            &hspi1,
            tx_buffer,
            rx,
            length,
            kSpiTimeoutMs);

    /* Release CS even when HAL reports an error. */
    release_all();

    s_last_status = status;

    if (status != HAL_OK)
    {
        ++s_error_count;
        return false;
    }

    return true;
}

extern "C" void BspBmi088_DelayUs(uint16_t us)
{
    delay_us(us);
}

extern "C" void BspBmi088_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

extern "C" uint32_t BspBmi088_GetSpiErrorCount(void)
{
    return s_error_count;
}

extern "C" uint32_t BspBmi088_GetLastHalStatus(void)
{
    return static_cast<uint32_t>(s_last_status);
}