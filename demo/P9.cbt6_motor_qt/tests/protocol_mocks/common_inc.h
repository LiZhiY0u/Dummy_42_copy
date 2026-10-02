#pragma once
#include <cstdint>
struct UART_HandleTypeDef { unsigned gState; };
enum { HAL_UART_STATE_READY = 0, HAL_OK = 0 };
extern UART_HandleTypeDef huart1;
uint32_t HAL_GetTick();
inline uint32_t HAL_GetUIDw0() { return 0x11223344; }
inline uint32_t HAL_GetUIDw1() { return 0x55667788; }
inline uint32_t HAL_GetUIDw2() { return 0x99aabbcc; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __enable_irq() {}
inline void __DMB() {}
struct FakeController {
    unsigned modeRunning = 0;
    void StopAndReset() { modeRunning = 0; }
    bool EnableProtocolMode(uint8_t, int32_t) { return true; }
    int32_t GetPositionSteps() const { return 0; }
    int32_t Get_est_velocity() const { return 0; }
    int32_t GetCurrentCommandMa() const { return 0; }
};
struct Motor {
    enum { MODE_STOP = 0, MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 51200 };
    FakeController *controller;
};
extern Motor motor;
struct FakeEncoder {
    struct { bool sampleValid = true; } angleData;
    bool IsCalibrated() const { return true; }
    uint16_t GetConsecutiveErrorCount() const { return 0; }
};
extern FakeEncoder mt6816_base;
struct FakeCalibrator { bool isTriggered = false; };
extern FakeCalibrator encoder_calibrator_base;
