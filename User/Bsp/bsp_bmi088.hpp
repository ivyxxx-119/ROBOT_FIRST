#ifndef BSP_BMI088_HPP
#define BSP_BMI088_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BSP_BMI088_ACCEL = 0,
        BSP_BMI088_GYRO = 1
    } BspBmi088Target;

    bool BspBmi088_Init(void);

    /*
     * One complete transaction:
     * assert CS -> transfer -> release CS.
     *
     * No internal RTOS mutex.
     * The caller must ensure exclusive use of SPI1.
     */
    bool BspBmi088_Transfer(
        BspBmi088Target target,
        const uint8_t *tx,
        uint8_t *rx,
        uint16_t length);

    void BspBmi088_DelayUs(uint16_t us);

    /* Blocking HAL delay; for startup initialization, not ISR use. */
    void BspBmi088_DelayMs(uint32_t ms);

    uint32_t BspBmi088_GetSpiErrorCount(void);
    uint32_t BspBmi088_GetLastHalStatus(void);

#ifdef __cplusplus
}
#endif

#endif