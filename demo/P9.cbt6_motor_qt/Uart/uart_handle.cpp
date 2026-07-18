#include "uart_handle.h"

uint8_t datalen = 0;
extern uint8_t rec_buff[100];
uint16_t rec = 0;
uint8_t Tx_flag = 0;

void Rx_handle(uint16_t size)
{
    if (size < 5)
        return;

    if (rec_buff[0] == 0xAA && rec_buff[1] == 0x55)
    {
        rec = (uint16_t)rec_buff[3] << 8 | rec_buff[4];
        if (rec_buff[2] == Motor::MODE_COMMAND_VELOCITY)
        {
            motor.controller->requestMode = (Motor::Mode_t)rec_buff[2];
            motor.controller->SetVelocitySetPoint((int32_t)((((int16_t)rec_buff[3] << 8 | rec_buff[4]) - 3000) * 512));
        }
        if (rec_buff[2] == Motor::MODE_COMMAND_POSITION)
        {
            motor.controller->requestMode = (Motor::Mode_t)rec_buff[2];
            motor.controller->SetPositionSetPoint((int32_t)((((uint16_t)rec_buff[3] << 8 | rec_buff[4]) - 5000) * 512));
        }
        if (rec_buff[2] == 0x22 && rec_buff[3] == 0x01)
        {
            Tx_flag = 1;
        }
        else if (rec_buff[2] == 0x22 && rec_buff[3] == 0x00)
        {
            Tx_flag = 0;
        }
    }
}

// 空闲处理中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart == &huart1)
    {
        datalen = (Size <= sizeof(rec_buff)) ? (uint8_t)Size : sizeof(rec_buff);
        Rx_handle(datalen);
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rec_buff, sizeof(rec_buff)) == HAL_OK)
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
uint8_t Tx_data[5] = {0xAA, 0x55, 0x22, 0x00, 0x00};
uint16_t Tx_velocity_data = 0;
void Upload_estvelocity()
{
    
    if (Tx_flag == 1)
    {
        Tx_velocity_data = (uint16_t)(motor.controller->GetVelocity() * 1000 + 30000);

        Tx_data[3] = (uint8_t)(Tx_velocity_data >> 8);
        Tx_data[4] = (uint8_t)Tx_velocity_data;
        if (huart1.gState == HAL_UART_STATE_READY)
            HAL_UART_Transmit_DMA(&huart1, &Tx_data[0], 5);
    }
}

void Upload_realposition()
{
    if (Tx_flag == 1)
    {
        uint16_t Tx_velocity_data = (uint16_t)(motor.controller->GetPosition());

        Tx_data[3] = (uint8_t)(Tx_velocity_data >> 8);
        Tx_data[4] = (uint8_t)Tx_velocity_data;
        if (huart1.gState == HAL_UART_STATE_READY)
            HAL_UART_Transmit_DMA(&huart1, &Tx_data[0], 5);
    }
}
