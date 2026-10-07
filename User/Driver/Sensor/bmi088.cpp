#include "bmi088.hpp"
#include "bsp_bmi088.hpp"
#include "debug.hpp"

/* ================================================================
 * 寄存器地址与配置值
 * ================================================================ */

/* 加速度计 */
#define REG_ACC_CHIP_ID 0x00U
#define ACC_CHIP_ID_VALUE 0x1EU
#define REG_ACC_PWR_CTRL 0x7DU
#define ACC_PWR_CTRL_ON 0x04U
#define REG_ACC_PWR_CONF 0x7CU
#define ACC_PWR_CONF_ACTIVE 0x00U
#define REG_ACC_CONF 0x40U
#define ACC_CONF_VALUE 0xABU /* NORMAL带宽 | 800Hz ODR */
#define REG_ACC_RANGE 0x41U
#define ACC_RANGE_3G 0x00U
#define REG_INT1_IO_CTRL 0x53U
#define INT1_IO_CTRL_VALUE 0x08U /* 输出使能 | 推挽 | 低有效 */
#define REG_INT_MAP_DATA 0x58U
#define INT_MAP_DATA_VALUE 0x04U /* DRDY映射到INT1 */
#define REG_ACC_SOFTRESET 0x7EU
#define ACC_SOFTRESET_VALUE 0xB6U
#define REG_ACC_XOUT_L 0x12U
#define REG_TEMP_M 0x22U

/* 陀螺仪 */
#define REG_GYRO_CHIP_ID 0x00U
#define GYRO_CHIP_ID_VALUE 0x0FU
#define REG_GYRO_RANGE 0x0FU
#define GYRO_RANGE_2000 0x00U
#define REG_GYRO_BW 0x10U
#define GYRO_BW_VALUE 0x82U /* 1000Hz ODR | 116Hz带宽 */
#define REG_GYRO_LPM1 0x11U
#define GYRO_NORMAL_MODE 0x00U
#define REG_GYRO_CTRL 0x15U
#define GYRO_DRDY_ON 0x80U
#define REG_GYRO_INT_CONF 0x16U
#define GYRO_INT_CONF_VALUE 0x00U /* 推挽 | 低有效 */
#define REG_GYRO_INT_MAP 0x18U
#define GYRO_DRDY_INT3 0x01U
#define REG_GYRO_SOFTRESET 0x14U
#define GYRO_SOFTRESET_VALUE 0xB6U

/* 物理量转换 */
#define ACCEL_SEN (0.0008974358974f) /* ±3g → m/s² */
#define GYRO_SEN (0.00106526443603f) /* ±2000°/s → rad/s */
#define TEMP_FACTOR (0.125f)
#define TEMP_OFFSET (23.0f)

/* 时序 */
#define RESET_WAIT_MS 80U
#define REG_WAIT_US 150U

namespace
{
    uint8_t s_init_error = 0xFFU;
    bool s_bus_failed = false;
}

static bool transfer(
    BspBmi088Target target,
    const uint8_t *tx,
    uint8_t *rx,
    uint16_t length)
{
    const bool ok =
        BspBmi088_Transfer(target, tx, rx, length);

    if (!ok)
    {
        s_bus_failed = true;
    }

    return ok;
}

static bool accel_write(uint8_t reg, uint8_t data)
{
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg & 0x7FU),
        data};
    uint8_t rx[2] = {};

    return transfer(BSP_BMI088_ACCEL, tx, rx, 2U);
}

static uint8_t accel_read(uint8_t reg)
{
    const uint8_t tx[3] = {
        static_cast<uint8_t>(reg | 0x80U),
        0x55U,
        0x55U};
    uint8_t rx[3] = {};

    if (!transfer(BSP_BMI088_ACCEL, tx, rx, 3U))
    {
        return 0U;
    }

    return rx[2];
}

static bool accel_read_burst(
    uint8_t reg,
    uint8_t *buf,
    uint8_t len)
{
    if (buf == nullptr || len == 0U || len > 8U)
    {
        return false;
    }

    uint8_t tx[10] = {};
    uint8_t rx[10] = {};

    tx[0] = static_cast<uint8_t>(reg | 0x80U);

    for (uint8_t i = 1U; i < len + 2U; ++i)
    {
        tx[i] = 0x55U;
    }

    if (!transfer(
            BSP_BMI088_ACCEL,
            tx,
            rx,
            static_cast<uint16_t>(len + 2U)))
    {
        return false;
    }

    for (uint8_t i = 0U; i < len; ++i)
    {
        buf[i] = rx[i + 2U];
    }

    return true;
}

static bool gyro_write(uint8_t reg, uint8_t data)
{
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg & 0x7FU),
        data};
    uint8_t rx[2] = {};

    return transfer(BSP_BMI088_GYRO, tx, rx, 2U);
}

static uint8_t gyro_read(uint8_t reg)
{
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg | 0x80U),
        0x55U};
    uint8_t rx[2] = {};

    if (!transfer(BSP_BMI088_GYRO, tx, rx, 2U))
    {
        return 0U;
    }

    return rx[1];
}

static bool gyro_read_burst(
    uint8_t reg,
    uint8_t *buf,
    uint8_t len)
{
    if (buf == nullptr || len == 0U || len > 8U)
    {
        return false;
    }

    uint8_t tx[9] = {};
    uint8_t rx[9] = {};

    tx[0] = static_cast<uint8_t>(reg | 0x80U);

    for (uint8_t i = 1U; i < len + 1U; ++i)
    {
        tx[i] = 0x55U;
    }

    if (!transfer(
            BSP_BMI088_GYRO,
            tx,
            rx,
            static_cast<uint16_t>(len + 1U)))
    {
        return false;
    }

    for (uint8_t i = 0U; i < len; ++i)
    {
        buf[i] = rx[i + 1U];
    }

    return true;
}

/* ================================================================
 * 写入并读回验证
 * ================================================================ */
static uint8_t write_verify(
    uint8_t is_accel,
    uint8_t reg,
    uint8_t val,
    uint8_t err_code)
{
    const bool write_ok =
        is_accel != 0U
            ? accel_write(reg, val)
            : gyro_write(reg, val);

    if (!write_ok)
    {
        return BMI088_ERR_BUS;
    }

    BspBmi088_DelayUs(REG_WAIT_US);

    const uint8_t rb =
        is_accel != 0U
            ? accel_read(reg)
            : gyro_read(reg);

    BspBmi088_DelayUs(REG_WAIT_US);

    if (s_bus_failed)
    {
        return BMI088_ERR_BUS;
    }

    return rb == val
               ? static_cast<uint8_t>(BMI088_OK)
               : err_code;
}
/* ================================================================
 * 加速度计初始化
 * ================================================================ */
static uint8_t accel_init(void)
{
    uint8_t id;
    uint8_t err;

    /* 上电第一次SPI访问结果无效，先读一次丢弃 */
    accel_read(REG_ACC_CHIP_ID);
    BspBmi088_DelayUs(REG_WAIT_US);

    /* 软件复位 */
    accel_write(REG_ACC_SOFTRESET, ACC_SOFTRESET_VALUE);
    BspBmi088_DelayMs(RESET_WAIT_MS);

    /* 复位后读两次，第二次才是真实ID */
    accel_read(REG_ACC_CHIP_ID);
    BspBmi088_DelayUs(REG_WAIT_US);
    id = accel_read(REG_ACC_CHIP_ID);
    BspBmi088_DelayUs(REG_WAIT_US);

    debugBmiAccelChipId = id;
    if (id != ACC_CHIP_ID_VALUE)
    {
        return BMI088_ERR_ACCEL_ID;
    }

    /* 开电源 */
    err = write_verify(1u, REG_ACC_PWR_CTRL,
                       ACC_PWR_CTRL_ON,
                       BMI088_ERR_ACCEL_PWR_CTRL);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayMs(RESET_WAIT_MS);

    /* 退出suspend */
    err = write_verify(1u, REG_ACC_PWR_CONF,
                       ACC_PWR_CONF_ACTIVE,
                       BMI088_ERR_ACCEL_PWR_CONF);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* 采样率800Hz，NORMAL带宽 */
    err = write_verify(1u, REG_ACC_CONF,
                       ACC_CONF_VALUE,
                       BMI088_ERR_ACCEL_CONF);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* 量程±3g */
    err = write_verify(1u, REG_ACC_RANGE,
                       ACC_RANGE_3G,
                       BMI088_ERR_ACCEL_RANGE);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* INT1输出配置 */
    err = write_verify(1u, REG_INT1_IO_CTRL,
                       INT1_IO_CTRL_VALUE,
                       BMI088_ERR_ACCEL_INT1);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* DRDY映射到INT1 */
    err = write_verify(1u, REG_INT_MAP_DATA,
                       INT_MAP_DATA_VALUE,
                       BMI088_ERR_ACCEL_INT_MAP);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    return BMI088_OK;
}

/* ================================================================
 * 陀螺仪初始化
 * ================================================================ */
static uint8_t gyro_init(void)
{
    uint8_t id;
    uint8_t err;

    /* 软件复位 */
    gyro_write(REG_GYRO_SOFTRESET, GYRO_SOFTRESET_VALUE);
    BspBmi088_DelayMs(RESET_WAIT_MS);

    /* 读ID两次确认 */
    gyro_read(REG_GYRO_CHIP_ID);
    BspBmi088_DelayUs(REG_WAIT_US);
    id = gyro_read(REG_GYRO_CHIP_ID);
    BspBmi088_DelayUs(REG_WAIT_US);

    debugBmiGyroChipId = id;
    if (id != GYRO_CHIP_ID_VALUE)
    {
        return BMI088_ERR_GYRO_ID;
    }

    /* 量程±2000°/s */
    err = write_verify(0u, REG_GYRO_RANGE,
                       GYRO_RANGE_2000,
                       BMI088_ERR_GYRO_RANGE);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* ODR 1000Hz，带宽116Hz */
    err = write_verify(0u, REG_GYRO_BW,
                       GYRO_BW_VALUE,
                       BMI088_ERR_GYRO_BW);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* 正常模式 */
    err = write_verify(0u, REG_GYRO_LPM1,
                       GYRO_NORMAL_MODE,
                       BMI088_ERR_GYRO_LPM);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* 使能DRDY中断输出 */
    err = write_verify(0u, REG_GYRO_CTRL,
                       GYRO_DRDY_ON,
                       BMI088_ERR_GYRO_CTRL);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* INT3推挽，低有效 */
    err = write_verify(0u, REG_GYRO_INT_CONF,
                       GYRO_INT_CONF_VALUE,
                       BMI088_ERR_GYRO_INT_CONF);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    /* DRDY映射到INT3 */
    err = write_verify(0u, REG_GYRO_INT_MAP,
                       GYRO_DRDY_INT3,
                       BMI088_ERR_GYRO_INT_MAP);
    if (err != BMI088_OK)
    {
        return err;
    }
    BspBmi088_DelayUs(REG_WAIT_US);

    return BMI088_OK;
}

/* ================================================================
 * 对外接口
 * ================================================================ */
uint8_t Bmi088_Init(void)
{
    s_init_error = 0xFFU;
    s_bus_failed = false;

    debugBmiAccelChipId = 0U;
    debugBmiGyroChipId = 0U;
    debugBmiAccelInitError = 0xFFU;
    debugBmiGyroInitError = 0xFFU;

    if (!BspBmi088_Init())
    {
        s_init_error = BMI088_ERR_PLATFORM;
        return s_init_error;
    }

    uint8_t err = accel_init();

    if (s_bus_failed)
    {
        err = BMI088_ERR_BUS;
    }

    debugBmiAccelInitError = err;

    if (err != BMI088_OK)
    {
        s_init_error = err;
        return err;
    }

    err = gyro_init();

    if (s_bus_failed)
    {
        err = BMI088_ERR_BUS;
    }

    debugBmiGyroInitError = err;

    s_init_error = err;
    return err;
}

uint8_t Bmi088_GetInitError(void)
{
    return s_init_error;
}

/* 将低字节在前的 16 位数据转换为有符号整数。 */
static int16_t decode_le16(const uint8_t *data)
{
    const uint16_t value =
        static_cast<uint16_t>(data[0]) |
        static_cast<uint16_t>(
            static_cast<uint16_t>(data[1]) << 8U);

    const int32_t signed_value =
        value >= 0x8000U
            ? static_cast<int32_t>(value) - 65536
            : static_cast<int32_t>(value);

    return static_cast<int16_t>(signed_value);
}

/*
 * 本次全部读取成功才更新输出。
 * 失败时返回 false，调用者的输出数组保持不变。
 */
bool Bmi088_TryRead(
    float gyro[3],
    float accel[3],
    float *temp)
{
    if (gyro == nullptr ||
        accel == nullptr ||
        temp == nullptr ||
        s_init_error != BMI088_OK)
    {
        return false;
    }

    uint8_t accel_buf[6] = {};
    uint8_t gyro_buf[8] = {};
    uint8_t temp_buf[2] = {};

    /* 加速度计：读取三轴数据。 */
    if (!accel_read_burst(
            REG_ACC_XOUT_L,
            accel_buf,
            6U))
    {
        return false;
    }

    /* 陀螺仪：从 ID 开始读取，包含三轴数据。 */
    if (!gyro_read_burst(
            REG_GYRO_CHIP_ID,
            gyro_buf,
            8U))
    {
        return false;
    }

    if (gyro_buf[0] != GYRO_CHIP_ID_VALUE)
    {
        return false;
    }

    /* 温度来自加速度计。 */
    if (!accel_read_burst(
            REG_TEMP_M,
            temp_buf,
            2U))
    {
        return false;
    }

    float new_accel[3] = {};
    float new_gyro[3] = {};

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        new_accel[i] =
            static_cast<float>(
                decode_le16(&accel_buf[2U * i])) *
            ACCEL_SEN;

        new_gyro[i] =
            static_cast<float>(
                decode_le16(&gyro_buf[2U + 2U * i])) *
            GYRO_SEN;
    }

    const uint16_t encoded_temp =
        static_cast<uint16_t>(
            (static_cast<uint16_t>(temp_buf[0]) << 3U) |
            (static_cast<uint16_t>(temp_buf[1]) >> 5U));

    const int32_t signed_temp =
        encoded_temp >= 1024U
            ? static_cast<int32_t>(encoded_temp) - 2048
            : static_cast<int32_t>(encoded_temp);

    const float new_temp =
        static_cast<float>(signed_temp) *
            TEMP_FACTOR +
        TEMP_OFFSET;

    /*
     * 到这里，本组通信已经全部成功。
     * 最后再统一更新调用者提供的输出。
     */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        gyro[i] = new_gyro[i];
        accel[i] = new_accel[i];
    }

    *temp = new_temp;

    return true;
}

/*
 * 旧接口兼容包装。
 * 不提供失败状态，新代码应使用 Bmi088_TryRead()。
 */
void Bmi088_Read(
    float gyro[3],
    float accel[3],
    float *temp)
{
    (void)Bmi088_TryRead(gyro, accel, temp);
}