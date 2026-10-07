#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

#include <stdint.h>



/* 原始驱动输出，保留原名称，方便Watch观察 */
extern float bmi_test_gyro[3];
extern float bmi_test_accel[3];
extern float bmi_test_temp;


/* 初始化及启动静止校准状态 */
extern volatile uint8_t bmi_test_ready;
extern volatile uint8_t bmi_test_bias_ready;

/*
 * 数据合理性检查状态。
 * 注意：不是完整的SPI通信健康证明。
 */
extern volatile uint8_t bmi_test_data_valid;
extern volatile uint32_t bmi_test_last_update_ms;

/* 车体坐标系数据 */
extern float bmi_body_accel[3];
extern float bmi_body_gyro[3];

/* 车体角速度，单位：度/秒 */
extern float bmi_roll_rate_dps;
extern float bmi_pitch_rate_dps;
extern float bmi_yaw_rate_dps;

/*
 * 加速度倾角参考，单位：度。
 * pitch：车头抬起为正。
 * roll：车体左侧抬起为正。
 *
 * 这是低动态倾角参考，不是完整姿态解算。
 */
extern float bmi_pitch_deg;
extern float bmi_roll_deg;
extern volatile uint8_t bmi_test_tilt_valid;

/* 启动阶段调用，必须在动力使能之前、车体静止时执行 */
void Imu_Init(void);

/* Robot_Task每轮调用，内部约10 ms读取一次，无osDelay */
void Imu_Update(void);

/* 当前数据是否可用于角速度辅助 */
uint8_t Imu_IsUsable(void);



#endif
