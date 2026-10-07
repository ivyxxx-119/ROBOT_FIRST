#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <stdint.h>

/*
 * 调试观测变量声明。
 * 所有定义集中在debug.cpp。
 */

extern volatile float debugBmiYawRateDps;
extern volatile float debugBmiStraightCorrectionRpm;
extern volatile uint8_t debugBmiStraightActive;
extern volatile uint32_t debugCan2LastRxId;
extern volatile uint32_t debugCan2RxCount;
extern volatile int16_t debugOut205;
extern volatile int16_t debugOut206;
extern volatile int16_t debugOut207;
extern volatile uint32_t debugRx205Count;
extern volatile uint32_t debugRx206Count;
extern volatile int16_t debugSpeed205;
extern volatile uint16_t debugAngle205;
extern volatile float debugSubTargetRpm0;
extern volatile float debugSubTargetRpm1;
extern volatile int16_t debugSubFeedRpm0;
extern volatile int16_t debugSubFeedRpm1;
extern volatile int16_t debugSubOutput0;
extern volatile int16_t debugSubOutput1;
extern volatile uint8_t debugSwitchRightRaw;
extern volatile uint8_t debugSwitchLeftRaw;
extern volatile uint8_t debugRobotState;
extern volatile float debugRightV;
extern volatile float debugLeftV;
extern volatile float debugDmTargetDeg;
extern volatile uint32_t debugUartRxEventCount;
extern volatile uint16_t debugUartRxSize;
extern volatile uint32_t debugDbusAcceptedCount;
extern volatile uint32_t debugDbusRejectedSizeCount;
extern volatile uint8_t debugRemoteOnline;
extern volatile uint32_t debugRemoteLastUpdateMs;
extern volatile uint32_t debugNowMs;
extern volatile uint32_t debugRemoteAgeMs;
extern volatile uint32_t debugTaskCount;
extern volatile uint8_t debugDmLeftOnline;
extern volatile uint8_t debugDmRightOnline;
extern volatile uint8_t debugDmLeftError;
extern volatile uint8_t debugDmRightError;
extern volatile float debugDmLeftAngleDeg;
extern volatile float debugDmRightAngleDeg;
extern volatile float debugDmLeftVelDps;
extern volatile float debugDmRightVelDps;
extern volatile float debugDmLeftTorque;
extern volatile float debugDmRightTorque;
extern volatile float debugDmLeftTargetDeg;
extern volatile float debugDmRightTargetDeg;
extern volatile uint32_t debugDmRxLeftCount;
extern volatile uint32_t debugDmRxRightCount;
extern volatile uint32_t debug201RxCount;
extern volatile uint32_t debug202RxCount;
extern volatile int16_t debug201SpeedRpm;
extern volatile int16_t debug202SpeedRpm;
extern volatile int16_t debugChTx201;
extern volatile int16_t debugChTx202;
extern volatile uint8_t bmi_test_init_error;
extern volatile uint32_t bmi_test_init_attempts;
extern volatile uint32_t bmi_test_read_count;
extern volatile uint8_t debugBmiAccelInitError;
extern volatile uint8_t debugBmiGyroInitError;

extern volatile uint8_t debugBmiAccelChipId;
extern volatile uint8_t debugBmiGyroChipId;
#endif
