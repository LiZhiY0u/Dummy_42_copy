#pragma once
#include "usart.h"

// Diagnostic only: call from the main loop, never from an ISR.
inline void UartTxTestStep()
{
    static uint8_t message[] = "UART1 TX TEST\r\n";
    (void)HAL_UART_Transmit(&huart1, message, sizeof(message) - 1, 100);
    HAL_Delay(500);
}
