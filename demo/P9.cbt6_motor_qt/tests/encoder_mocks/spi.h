#pragma once
#include <cstdint>
extern uint32_t encoderMetadata[5];
extern uint16_t encoderFlashKib;
#define FLASHSIZE_BASE reinterpret_cast<uintptr_t>(&encoderFlashKib)
#define P9_CALIBRATION_RECORD_ADDRESS reinterpret_cast<uintptr_t>(encoderMetadata)
#define __IO volatile
// Test-only register double: RXNE reads can supply a scripted wire response.
struct ScriptedStatus {
    uint32_t flags = 0;
    uint32_t *data;
    uint16_t response[16] = {};
    unsigned count = 0, cursor = 0;
    explicit ScriptedStatus(uint32_t *value) : data(value) {}
    ScriptedStatus &operator=(uint32_t value) { flags=value; return *this; }
    uint32_t operator&(uint32_t mask) {
        if (mask == 1 && (flags & 1) && cursor < count)
            *data = response[cursor++];
        return flags & mask;
    }
    void raw(uint16_t a, uint16_t b, uint16_t c) {
        response[0]=a>>8; response[1]=a&255;
        response[2]=b>>8; response[3]=b&255;
        response[4]=c>>8; response[5]=c&255;
        count=6; cursor=0;
    }
};
struct SPI_TypeDef {
    uint32_t CR1 = 0;
    ScriptedStatus SR;
    uint32_t DR = 0;
    SPI_TypeDef() : SR(&DR) {}
};
struct SPI_HandleTypeDef { SPI_TypeDef *Instance; };
struct GPIO_TypeDef { uint32_t BSRR = 0, BRR = 0; };
extern SPI_HandleTypeDef hspi1;
extern GPIO_TypeDef *GPIOA;
enum { GPIO_PIN_15 = 1 << 15, SPI_SR_TXE = 2, SPI_SR_RXNE = 1,
       SPI_SR_BSY = 128, SPI_CR1_SPE = 64 };
inline void MX_SPI1_Init() {}
#define __HAL_SPI_ENABLE(handle) ((handle)->Instance->CR1 |= SPI_CR1_SPE)
