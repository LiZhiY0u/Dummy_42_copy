#pragma once
#include <cstdint>
#define __IO volatile
struct SPI_TypeDef { uint32_t CR1 = 0, SR = 0, DR = 0; };
struct SPI_HandleTypeDef { SPI_TypeDef *Instance; };
struct GPIO_TypeDef { uint32_t BSRR = 0, BRR = 0; };
extern SPI_HandleTypeDef hspi1;
extern GPIO_TypeDef *GPIOA;
enum { GPIO_PIN_15 = 1 << 15, SPI_SR_TXE = 2, SPI_SR_RXNE = 1,
       SPI_SR_BSY = 128, SPI_CR1_SPE = 64 };
inline void MX_SPI1_Init() {}
#define __HAL_SPI_ENABLE(handle) ((handle)->Instance->CR1 |= SPI_CR1_SPE)
