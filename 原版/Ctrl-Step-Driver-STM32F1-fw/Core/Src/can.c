/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "common_inc.h"
#include "configurations.h"
#include <string.h>

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    CAN_FilterTypeDef sFilterConfig;
    //filter one (stack light blink)
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    HAL_CAN_Start(&hcan); //start CAN

    HAL_CAN_ActivateNotification(&hcan,
                                 CAN_IT_TX_MAILBOX_EMPTY |
                                 CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
                                 /* we probably only want this */
                                 CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO1_FULL |
                                 CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN |
                                 CAN_IT_WAKEUP | CAN_IT_SLEEP_ACK |
                                 CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                 CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                 CAN_IT_ERROR);

/* Configure Transmission process */
    TxHeader.StdId = boardConfig.canNodeId;
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
    // Replies are best effort: a full mailbox must not halt the control loop.
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0U)
        (void)HAL_CAN_AddTxMessage(&hcan, pHeader, data, &TxMailbox);
}

// Single-producer/single-consumer queue. Drop new frames on overload.
static struct { uint16_t id; uint8_t len; uint8_t data[8]; } pending[8];
static volatile uint8_t head = 0, tail = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};
    if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8) return;
    uint8_t next = (head + 1U) % 8U;
    if (next == tail) return;
    pending[head].id = header.StdId;
    pending[head].len = header.DLC;
    memcpy(pending[head].data, data, 8);
    __DMB();
    head = next;
}

void CAN_ProcessPending(void)
{
    if (head == tail) return;
    uint16_t id = pending[tail].id;
    uint8_t len = pending[tail].len;
    uint8_t data[8];
    memcpy(data, pending[tail].data, 8);
    __DMB();
    tail = (tail + 1U) % 8U;
    if ((id >> 7) == 0 || (id >> 7) == boardConfig.canNodeId)
        OnCanCmd(id & 0x7f, data, len);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
