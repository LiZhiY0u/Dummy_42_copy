#pragma once
#include <cstdint>
extern uint16_t protocolFlashKib;
#define FLASHSIZE_BASE reinterpret_cast<uintptr_t>(&protocolFlashKib)
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
    unsigned stops = 0, enables = 0;
    void StopAndReset() { ++stops; modeRunning = 0; }
    bool EnableProtocolMode(uint8_t, int32_t) { ++enables; return true; }
    int32_t GetPositionSteps() const { return 0; }
    int32_t Get_est_velocity() const { return 0; }
    int32_t GetCurrentCommandMa() const { return 0; }
};
struct Motor {
    enum { MODE_STOP = 0, MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 51200 };
    FakeController *controller;
    struct { struct { int32_t caliCurrent = 1000; } motionParams; } config;
    Motor(FakeController *value) : controller(value) {}
};
extern Motor motor;
struct FakeEncoder {
    struct { bool sampleValid = true; } angleData;
    bool calibrated = true;
    bool IsCalibrated() const { return calibrated; }
    uint16_t GetConsecutiveErrorCount() const { return 0; }
};
extern FakeEncoder mt6816_base;
struct FakeCalibrator {
    enum { CALI_DISABLE=0, CALI_FORWARD_PREPARE=1, CALI_CALCULATING=6, CALI_NO_ERROR=0 };
    bool isTriggered = false;
    unsigned state = CALI_DISABLE;
    unsigned errorCode=0, goPosition=0, sampleCount=0, resultNum=0;
    int rcdX=0, rcdY=0;
};
extern FakeCalibrator encoder_calibrator_base;
typedef FakeCalibrator EncoderCalibratorBase;
