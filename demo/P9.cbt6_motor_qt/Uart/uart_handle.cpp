#include "uart_handle.h"
#include "uart_protocol.h"

extern uint8_t rec_buff[160];

namespace
{
void restartReceive()
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rec_buff, sizeof(rec_buff)) == HAL_OK)
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart1)
        return;
    if (size <= sizeof(rec_buff))
        UartProtocolOnRx(rec_buff, size);
    restartReceive();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        UartProtocolOnTxComplete();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
        return;
    UartProtocolOnError();
    HAL_UART_AbortReceive(huart);
    restartReceive();
}

void Upload_estvelocity()
{
    // Protocol V1 telemetry is scheduled from the main loop.
}
