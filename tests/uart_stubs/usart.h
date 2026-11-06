#pragma once
#include <cstdint>
constexpr unsigned BUFFER_SIZE=128;
constexpr int HAL_UART_STATE_READY=0;
struct UART_HandleTypeDef { int gState = 0, RxState = 1; };
extern UART_HandleTypeDef huart1;
extern uint8_t rx_buffer[BUFFER_SIZE];
uint32_t HAL_GetTick();
int HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
