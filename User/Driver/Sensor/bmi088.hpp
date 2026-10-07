#ifndef BMI088_HPP
#define BMI088_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BMI088_OK = 0x00U,

        BMI088_ERR_ACCEL_ID = 0x01U,
        BMI088_ERR_ACCEL_PWR_CTRL = 0x02U,
        BMI088_ERR_ACCEL_PWR_CONF = 0x03U,
        BMI088_ERR_ACCEL_CONF = 0x04U,
        BMI088_ERR_ACCEL_RANGE = 0x05U,
        BMI088_ERR_ACCEL_INT1 = 0x06U,
        BMI088_ERR_ACCEL_INT_MAP = 0x07U,

        BMI088_ERR_GYRO_ID = 0x10U,
        BMI088_ERR_GYRO_RANGE = 0x11U,
        BMI088_ERR_GYRO_BW = 0x12U,
        BMI088_ERR_GYRO_LPM = 0x13U,
        BMI088_ERR_GYRO_CTRL = 0x14U,
        BMI088_ERR_GYRO_INT_CONF = 0x15U,
        BMI088_ERR_GYRO_INT_MAP = 0x16U,

        BMI088_ERR_BUS = 0x20U,
        BMI088_ERR_PLATFORM = 0x21U
    } Bmi088Error_t;

    uint8_t Bmi088_Init(void);
    uint8_t Bmi088_GetInitError(void);

    /*
     * Outputs are updated only when the entire sample is read successfully.
     */
    bool Bmi088_TryRead(
        float gyro[3],
        float accel[3],
        float *temp);

    /* Compatibility interface. New code should use Bmi088_TryRead(). */
    void Bmi088_Read(
        float gyro[3],
        float accel[3],
        float *temp);

#ifdef __cplusplus
}
#endif

#endif