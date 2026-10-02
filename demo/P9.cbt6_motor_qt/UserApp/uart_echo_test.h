#pragma once
#include "usart.h"

// Wiring diagnostic only; blocking calls run in the main loop, never an ISR.
inline void UartEchoTestStep()
{
    uint8_t byte = 0;
    if (HAL_UART_Receive(&huart1, &byte, 1, 100) == HAL_OK)
        (void)HAL_UART_Transmit(&huart1, &byte, 1, 100);
}
