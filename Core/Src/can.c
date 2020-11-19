/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usbd_cdc_if.h"
#include "canopen_object_dict.h"

/************************************************************************************************
 GLOBAL VARIABLES
 ************************************************************************************************/
CanDataFrameInit can_frame_template;
CanDataFrameInit *p_can_frame_template = &can_frame_template;
CAN_FilterTypeDef can_filter_template;
CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];
CanDataFrameInit can_rx_frame_template;
uint32_t can_tx_mailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 21;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/************************************************************************************************
 FUNCTIONS CREATED OUTSIDE CUBEMX
 ************************************************************************************************/
void CanInit(CAN_HandleTypeDef hcanx) {
	if (HAL_CAN_Start(&hcanx) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcanx,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}
}

void CanSaveReceivedData(CAN_HandleTypeDef hcanx) {
	if (HAL_CAN_GetRxMessage(&hcanx, CAN_RX_FIFO0, &can_rx_header, can_rx_data)
			!= HAL_OK) {
		/* Reception Error */
		Error_Handler();
	}
}

/*
 * @brief Configure CAN data frames hardware filter
 *
 * @param hcanx: Network choice
 * @param can_filter_bank: Specific filter bank
 * @param can_filter_id_high: High byte of CAN ID to be received
 * @param can_filter_id_low: Low byte of CAN ID to be received
 * @param can_filter_mask_id_high: High byte of CAN ID mask - IDs to be received
 * @param can_filter_mask_id_low: Low byte of CAN ID mask - IDs to be received
 */
void CanConfigFilter(CAN_HandleTypeDef hcanx, uint8_t can_filter_bank,
		uint32_t can_filter_id_high, uint32_t can_filter_id_low,
		uint32_t can_filter_mask_id_high, uint32_t can_filter_mask_id_low) {
	can_filter_template.FilterBank = can_filter_bank;
	can_filter_template.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_template.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_template.FilterIdHigh = can_filter_id_high; //18FF;			//0x321 << 5;
	can_filter_template.FilterIdLow = can_filter_id_low; //50E5;				//0x00000000;
	can_filter_template.FilterMaskIdHigh = can_filter_mask_id_high;	//0x111 << 5;
	can_filter_template.FilterMaskIdLow = can_filter_mask_id_low;//0x00000000;
	can_filter_template.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter_template.FilterActivation = ENABLE;
	can_filter_template.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcanx, &can_filter_template) != HAL_OK) {
		Error_Handler();
	}

}

// Send CANopen SYNC data frame
void CanSendSync(CAN_HandleTypeDef hcanx, CanDataFrameInit *can_frame_template) {
	can_frame_template->tx_header.StdId = 0x080;
	can_frame_template->tx_header.RTR = CAN_RTR_DATA;
	can_frame_template->tx_header.IDE = CAN_ID_STD;
	can_frame_template->tx_header.DLC = 0;
	can_frame_template->tx_header.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcanx, &can_frame_template->tx_header,
			can_frame_template->tx_data, &can_tx_mailbox) != HAL_OK) {
		Error_Handler();
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcanx) != 3) {
	}

}

// Send CANopen NMT data frame
void CanSendNmt(CAN_HandleTypeDef hcanx, uint8_t state, uint8_t node_id,
		CanDataFrameInit *can_frame_template) {
	can_frame_template->tx_header.StdId = 0x000;
	can_frame_template->tx_header.RTR = CAN_RTR_DATA;
	can_frame_template->tx_header.IDE = CAN_ID_STD;
	can_frame_template->tx_header.DLC = 2;
	can_frame_template->tx_header.TransmitGlobalTime = DISABLE;

	can_frame_template->tx_data[0] = state;
	can_frame_template->tx_data[1] = node_id;

	UsbTransfer(can_frame_template);

	if (HAL_CAN_AddTxMessage(&hcanx, &can_frame_template->tx_header,
			can_frame_template->tx_data, &can_tx_mailbox) != HAL_OK) {
		Error_Handler();
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcanx) != 3) {
	}

}

// Send CANopen TPDO data frame
void CanSendTpdo(CAN_HandleTypeDef hcanx, uint8_t node_id, uint8_t data_1,
		CanDataFrameInit *can_frame_template) {
	can_frame_template->tx_header.StdId = node_id;
	can_frame_template->tx_header.RTR = CAN_RTR_DATA;
	can_frame_template->tx_header.IDE = CAN_ID_STD;
	can_frame_template->tx_header.DLC = 8;
	can_frame_template->tx_header.TransmitGlobalTime = DISABLE;

	can_frame_template->tx_data[0] = 0x1F;
	can_frame_template->tx_data[1] = 0x23;
	can_frame_template->tx_data[2] = 0x78;
	can_frame_template->tx_data[3] = 0xFF;
	can_frame_template->tx_data[4] = 0x2F;
	can_frame_template->tx_data[5] = 0x1C;
	can_frame_template->tx_data[6] = 0x00;
	can_frame_template->tx_data[7] = data_1;

	if (HAL_CAN_AddTxMessage(&hcanx, &can_frame_template->tx_header,
			can_frame_template->tx_data, &can_tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcanx) != 3) {
	}
}

void CanSendTpdoTest(CAN_HandleTypeDef hcanx) {
	/* moge rowniez napisac wskaznik na strukture, w tym celu oszczedze sporo pamieci. instrukcje sa na 17.8 w C wikibooks */
	/* moge napisac funkcje uniwersalna  */
	/* no i oczywiscie dynamiczna alokacje pamieci */
	can_frame_template.tx_header.StdId = 0x581;
	can_frame_template.tx_header.RTR = CAN_RTR_DATA;
	can_frame_template.tx_header.IDE = CAN_ID_STD;
	can_frame_template.tx_header.DLC = 8;
	can_frame_template.tx_header.TransmitGlobalTime = DISABLE;

	can_frame_template.tx_data[0] = 0xDE;
	can_frame_template.tx_data[1] = 0xAD;
	can_frame_template.tx_data[2] = 0xBE;
	can_frame_template.tx_data[3] = 0xEF;
	can_frame_template.tx_data[4] = 0xDE;
	can_frame_template.tx_data[5] = 0xAD;
	can_frame_template.tx_data[6] = 0xBE;
	can_frame_template.tx_data[7] = 0xEF;

	if (HAL_CAN_AddTxMessage(&hcanx, &can_frame_template.tx_header,
			can_frame_template.tx_data, &can_tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
	}

}
/**
 * @brief Transfer specific CAN data frame to selected low or high speed network
 * @param hcanx: chosen network
 * @param sender_id: received ID
 * @param receiver_id: node chosen to receive
 *
 * @notes 14/09 czy nie moge po prostu przeslac calej tablicy?
 */
void CanTransfer(CAN_HandleTypeDef hcanx, uint32_t sender_id,
		uint32_t receiver_id) {
	if ((can_rx_header.StdId == sender_id)
			&& (can_rx_header.RTR == CAN_RTR_DATA)
			&& (can_rx_header.IDE == CAN_ID_STD)) {
		for (int i = 0; i <= can_rx_header.DLC - 1; i++) {
			can_rx_frame_template.rx_data[i] = can_rx_data[i];
		}
	} else {
		HAL_GPIO_TogglePin(LED_D6_GPIO_Port, LED_D6_Pin);
	}

	if (HAL_CAN_AddTxMessage(&hcanx, &can_rx_frame_template.tx_header,
			can_rx_frame_template.tx_data, &can_tx_mailbox) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
