#include "imu_service.hpp"
#include "debug.hpp"
#include "bmi088.hpp"
#include "main.h"

#include <cmath>
#include <stdint.h>

#define BMI_RAD_TO_DEG 57.2957795131f
#define BMI_GRAVITY 9.80665f

#define BMI_POLL_MS 10U
#define BMI_STALE_MS 100U

#define BMI_CALIB_SAMPLES 100U
#define BMI_CALIB_MAX_TRIES 300U

/* 原始驱动输出 */
float bmi_test_gyro[3] = {0.0f};
float bmi_test_accel[3] = {0.0f};
float bmi_test_temp = 0.0f;

volatile uint8_t bmi_test_ready = 0U;
volatile uint8_t bmi_test_bias_ready = 0U;
volatile uint8_t bmi_test_data_valid = 0U;
volatile uint8_t bmi_test_tilt_valid = 0U;
volatile uint32_t bmi_test_last_update_ms = 0U;

/* 车体坐标数据 */
float bmi_body_accel[3] = {0.0f};
float bmi_body_gyro[3] = {0.0f};

float bmi_roll_rate_dps = 0.0f;
float bmi_pitch_rate_dps = 0.0f;
float bmi_yaw_rate_dps = 0.0f;

float bmi_pitch_deg = 0.0f;
float bmi_roll_deg = 0.0f;

/* 内部状态 */
static float gyro_bias[3] = {0.0f};

static uint32_t last_poll_ms = 0U;
static uint32_t last_filter_ms = 0U;
static uint8_t filter_started = 0U;
static uint8_t tilt_started = 0U;

static float vector_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/*
 * 传感器坐标转换为车体坐标。
 *
 * 当前对应：
 * 正面朝上，KEY/UART6端朝车头。
 *
 * 如果实际安装不同，只修改这个函数。
 * 加速度和陀螺仪必须使用同一个正确的坐标变换。
 */
static void sensor_to_body(const float sensor[3], float body[3])
{
    body[0] = sensor[0];
    body[1] = sensor[1];
    body[2] = sensor[2];
}

/*
 * 这里只检查数值是否有限以及重力模长是否明显异常。
 * 无法检测所有SPI故障，例如冻结在一组看似合理的数据。
 */
static uint8_t raw_values_plausible(void)
{
    for (uint32_t i = 0U; i < 3U; i++)
    {
        if (!std::isfinite(bmi_test_gyro[i]) ||
            !std::isfinite(bmi_test_accel[i]))
        {
            return 0U;
        }
    }

    if (!std::isfinite(bmi_test_temp))
    {
        return 0U;
    }

    float accel_norm = vector_norm(bmi_test_accel);

    /* 排除全零及严重异常值，不用它判断倾角可信度 */
    if (accel_norm < 1.0f || accel_norm > 50.0f)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 启动时采集连续静止样本，估计陀螺仪零偏。
 *
 * 必须保持车体静止、动力未使能。
 * 判断条件只是基本筛选，不能识别所有缓慢运动。
 */
static void calibrate_gyro_bias(void)
{
    float sum[3] = {0.0f};
    uint32_t count = 0U;

    bmi_test_bias_ready = 0U;

    for (uint32_t attempt = 0U;
         attempt < BMI_CALIB_MAX_TRIES;
         ++attempt)
    {
        const bool read_ok =
            Bmi088_TryRead(
                bmi_test_gyro,
                bmi_test_accel,
                &bmi_test_temp);

        bool stationary = false;

        /*
         * 必须本次读取成功，才能检查并使用这组数据。
         * 失败时不能拿数组中的旧值继续校准。
         */
        if (read_ok && raw_values_plausible())
        {
            const float accel_norm =
                vector_norm(bmi_test_accel);

            const float gyro_norm =
                vector_norm(bmi_test_gyro);

            stationary =
                fabsf(accel_norm - BMI_GRAVITY) < 0.8f &&
                gyro_norm < 0.10f;
        }

        if (stationary)
        {
            for (uint32_t i = 0U; i < 3U; ++i)
            {
                sum[i] += bmi_test_gyro[i];
            }

            ++count;

            if (count >= BMI_CALIB_SAMPLES)
            {
                for (uint32_t i = 0U; i < 3U; ++i)
                {
                    gyro_bias[i] =
                        sum[i] / static_cast<float>(count);
                }

                bmi_test_bias_ready = 1U;
                return;
            }
        }
        else
        {
            /*
             * 要求连续的成功且静止样本。
             * 通信失败、数据不合理或检测到运动，都重新开始。
             */
            count = 0U;

            for (uint32_t i = 0U; i < 3U; ++i)
            {
                sum[i] = 0.0f;
            }
        }

        /*
         * 无论本次成功还是失败，都保留采样间隔。
         * 当前校准设计用于调度器启动前的初始化阶段。
         */
        HAL_Delay(BMI_POLL_MS);
    }

    /*
     * 超过尝试次数仍未完成校准。
     * bmi_test_bias_ready 保持为 0。
     */
}

void Imu_Init(void)
{
    bmi_test_ready = 0U;
    bmi_test_bias_ready = 0U;
    bmi_test_data_valid = 0U;
    bmi_test_tilt_valid = 0U;

    bmi_test_init_error = 0xFF;
    bmi_test_init_attempts = 0U;
    bmi_test_read_count = 0U;
    bmi_test_last_update_ms = 0U;

    filter_started = 0U;
    tilt_started = 0U;

    bmi_roll_rate_dps = 0.0f;
    bmi_pitch_rate_dps = 0.0f;
    bmi_yaw_rate_dps = 0.0f;

    bmi_pitch_deg = 0.0f;
    bmi_roll_deg = 0.0f;

    for (uint32_t i = 0U; i < 3U; i++)
    {
        gyro_bias[i] = 0.0f;
    }

    /* 有限次数重试，不无限阻塞整车启动 */
    for (uint32_t i = 0U; i < 5U; i++)
    {
        bmi_test_init_attempts++;

        bmi_test_init_error = Bmi088_Init();

        if (bmi_test_init_error == 0U)
        {
            bmi_test_ready = 1U;
            break;
        }

        if (i < 4U)
        {
            HAL_Delay(100U);
        }
    }

    if (bmi_test_ready)
    {
        calibrate_gyro_bias();
    }

    last_poll_ms = HAL_GetTick();
    last_filter_ms = last_poll_ms;
}

void Imu_Update(void)
{
    if (!bmi_test_ready)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_poll_ms) < BMI_POLL_MS)
    {
        return;
    }

    last_poll_ms = now_ms;

    if (!Bmi088_TryRead(
            bmi_test_gyro,
            bmi_test_accel,
            &bmi_test_temp))
    {
        bmi_test_data_valid = 0U;
        bmi_test_tilt_valid = 0U;

        filter_started = 0U;
        tilt_started = 0U;

        return;
    }

    /* 现在统计的是成功读取次数。 */
    ++bmi_test_read_count;

    if (!raw_values_plausible())
    {
        bmi_test_data_valid = 0U;
        bmi_test_tilt_valid = 0U;

        filter_started = 0U;
        tilt_started = 0U;
        return;
    }

    float corrected_gyro[3];

    for (uint32_t i = 0U; i < 3U; i++)
    {
        corrected_gyro[i] =
            bmi_test_gyro[i] - gyro_bias[i];
    }

    sensor_to_body(bmi_test_accel, bmi_body_accel);
    sensor_to_body(corrected_gyro, bmi_body_gyro);

    uint32_t sample_ms = HAL_GetTick();
    uint32_t elapsed_ms = sample_ms - last_filter_ms;

    last_filter_ms = sample_ms;

    /* 长时间中断后，不沿用旧滤波状态 */
    if (elapsed_ms > BMI_STALE_MS)
    {
        filter_started = 0U;
        tilt_started = 0U;
    }

    float dt = (float)elapsed_ms * 0.001f;

    /* 角速度低通滤波，时间常数约30 ms */
    float rate_alpha = dt / (0.03f + dt);

    float roll_rate =
        bmi_body_gyro[0] * BMI_RAD_TO_DEG;

    /* 显示约定：抬头为正 */
    float pitch_rate =
        -bmi_body_gyro[1] * BMI_RAD_TO_DEG;

    float yaw_rate =
        bmi_body_gyro[2] * BMI_RAD_TO_DEG;

    if (!filter_started)
    {
        bmi_roll_rate_dps = roll_rate;
        bmi_pitch_rate_dps = pitch_rate;
        bmi_yaw_rate_dps = yaw_rate;

        filter_started = 1U;
    }
    else
    {
        bmi_roll_rate_dps += rate_alpha *
                             (roll_rate - bmi_roll_rate_dps);

        bmi_pitch_rate_dps += rate_alpha *
                              (pitch_rate - bmi_pitch_rate_dps);

        bmi_yaw_rate_dps += rate_alpha *
                            (yaw_rate - bmi_yaw_rate_dps);
    }

    /*
     * 加速度倾角参考：
     * 仅在接近重力模长、车体转动较慢且大致正立时更新。
     *
     * 即便满足条件，也不保证没有运动加速度干扰。
     */
    float accel_norm = vector_norm(bmi_body_accel);
    float gyro_norm = vector_norm(bmi_body_gyro);

    bmi_test_tilt_valid = 0U;

    if (fabsf(accel_norm - BMI_GRAVITY) < 0.8f &&
        gyro_norm < 0.35f &&
        bmi_body_accel[2] > 0.0f)
    {
        float ax = bmi_body_accel[0];
        float ay = bmi_body_accel[1];
        float az = bmi_body_accel[2];

        float pitch = atan2f(
                          ax,
                          sqrtf(ay * ay + az * az)) *
                      BMI_RAD_TO_DEG;

        float roll = atan2f(ay, az) * BMI_RAD_TO_DEG;

        /* 倾角低通滤波，时间常数约200 ms */
        float tilt_alpha = dt / (0.20f + dt);

        if (!tilt_started)
        {
            bmi_pitch_deg = pitch;
            bmi_roll_deg = roll;

            tilt_started = 1U;
        }
        else
        {
            bmi_pitch_deg += tilt_alpha *
                             (pitch - bmi_pitch_deg);

            bmi_roll_deg += tilt_alpha *
                            (roll - bmi_roll_deg);
        }

        bmi_test_tilt_valid = 1U;
    }
    else
    {
        /* 下次可信时从新倾角开始，避免沿用旧姿态 */
        tilt_started = 0U;
    }

    bmi_test_data_valid = 1U;
    bmi_test_last_update_ms = sample_ms;
}

uint8_t Imu_IsUsable(void)
{
    if (!bmi_test_ready ||
        !bmi_test_bias_ready ||
        !bmi_test_data_valid)
    {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() -
                   bmi_test_last_update_ms) > BMI_STALE_MS)
    {
        return 0U;
    }

    return 1U;
}
