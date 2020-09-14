/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/*******************************************************************************
 DEFINES
 *******************************************************************************/
#define CAN_HIGH_SPEED hcan1
#define CAN_LOW_SPEED hcan2
#define OPERATIONAL_STATE		0x01
#define STOPPED_STATE			0x02
#define PRE_OPERATIONAL_STATE	0x80
#define RESET_APPLICATION		0x81
#define RESET_COMMUNICATION		0x82

/*******************************************************************************
 DECLARATIONS
 *******************************************************************************/
typedef struct {
	uint8_t tx_data[8];
	CAN_TxHeaderTypeDef tx_header;
	uint8_t rx_data[8];
} CanDataFrameInit;

void CanSendSync(CAN_HandleTypeDef hcanx);
void CanSendNmt(CAN_HandleTypeDef hcanx, uint8_t state, uint8_t node_id);
void CanSendTpdo(CAN_HandleTypeDef hcanx, int32_t node_id, uint32_t data1[8]);
void CanSendTpdoTest(CAN_HandleTypeDef hcanx);

void CanInit(CAN_HandleTypeDef hcanx);
void CanConfigFilter(CAN_HandleTypeDef hcanx, uint8_t can_filter_bank,
		uint32_t can_filter_id_high, uint32_t can_filter_id_low,
		uint32_t can_filter_mask_id_high, uint32_t can_filter_mask_id_low);

void CanSaveReceivedData(CAN_HandleTypeDef hcanx);
void CanTransfer(CAN_HandleTypeDef hcanx, uint32_t sender_id,
		uint32_t receiver_id);

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
