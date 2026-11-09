#pragma once
#include <cstdint>
#include "stockpile_f103cb.h"
struct UART_HandleTypeDef { unsigned gState; };
enum { HAL_UART_STATE_READY=0, HAL_OK=0 };
extern UART_HandleTypeDef huart1;
uint32_t HAL_GetTick();
inline uint32_t HAL_GetUIDw0(){return 1;}
inline uint32_t HAL_GetUIDw1(){return 2;}
inline uint32_t HAL_GetUIDw2(){return 3;}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
inline void __enable_irq(){}
inline void __DMB(){}
struct IntegrationController {
    unsigned modeRunning=0,stops=0,enables=0;
    void StopAndReset(){modeRunning=0;++stops;}
    bool EnableProtocolMode(uint8_t,int32_t){++enables;return true;}
    int32_t GetPositionSteps()const{return 0;}
    int32_t Get_est_velocity()const{return 0;}
    int32_t GetCurrentCommandMa()const{return 0;}
};
struct Motor {
    enum {MODE_STOP=0,MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS=51200};
    IntegrationController *controller;
    struct {struct {int32_t caliCurrent=1000;}motionParams;}config;
    explicit Motor(IntegrationController *p):controller(p){}
};
extern Motor motor;
#include "mt6816_base.h"
#include "encoder_calibrator_base.h"
