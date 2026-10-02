#include <cassert>
#include <cstdio>
#include "uart_echo_test.h"
UART_HandleTypeDef huart1;
static const uint8_t input[] = {'1', 'A', 0, 0xAA, 0xFF};
static unsigned receivedCount, sentCount;
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                 uint16_t size, uint32_t timeout)
{
    assert(uart == &huart1 && size == 1 && timeout == 100);
    if (receivedCount == sizeof(input))
        return HAL_TIMEOUT;
    *data = input[receivedCount++];
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t size, uint32_t timeout)
{
    assert(uart == &huart1 && size == 1 && timeout == 100);
    assert(sentCount < sizeof(input) && *data == input[sentCount++]);
    return HAL_OK;
}
int main()
{
    for (unsigned i = 0; i < sizeof(input) + 2; ++i)
        UartEchoTestStep();
    assert(sentCount == sizeof(input));
    std::puts("PASS: exact byte echo including zero/FF; no output on RX timeout");
}
