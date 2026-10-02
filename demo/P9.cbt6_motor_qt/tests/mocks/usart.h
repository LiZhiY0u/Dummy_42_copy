#pragma once
#include <cstdint>
struct UART_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_TIMEOUT };
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
