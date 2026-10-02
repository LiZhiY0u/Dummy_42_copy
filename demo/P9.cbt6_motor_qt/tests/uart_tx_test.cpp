#include <cassert>
#include <cstring>
#include <cstdio>
#include "uart_tx_test.h"

UART_HandleTypeDef huart1;
static unsigned transmitCount;
static unsigned delayCount;
static unsigned order;
static HAL_StatusTypeDef nextStatus = HAL_OK;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    assert(uart == &huart1);
    assert(size == 15);
    assert(std::memcmp(data, "UART1 TX TEST\r\n", 15) == 0);
    assert(timeout == 100);
    assert(order == 0);
    order = 1;
    ++transmitCount;
    return nextStatus;
}

void HAL_Delay(uint32_t milliseconds)
{
    assert(milliseconds == 500);
    assert(order == 1);
    order = 0;
    ++delayCount;
}

int main()
{
    UartTxTestStep();
    UartTxTestStep();
    nextStatus = HAL_ERROR;
    UartTxTestStep(); // A failed transmit must still delay before retrying.
    assert(transmitCount == 3 && delayCount == 3 && order == 0);
    std::puts("PASS: repeated UART1 text, bounded timeout, delay on error");
}
