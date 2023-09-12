/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
UART_HandleTypeDef huart2;	//whu
static CO_Data *co_data = NULL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//FDCAN_FilterTypeDef sFilterConfig;
 
 FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = {0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};
 FDCAN_RxHeaderTypeDef RxHeader;
 uint8_t RxData[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */
//	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
//	
////	if (FDCAN_IT_RX_FIFO0_NEW_MESSAGE != RESET)
////	{
////		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) !=HAL_OK)
////		{
////			//Error_Handler();
////			printf("FDCAN_RX_Error.");
////		}
////	}

	int i;
//CanRxMsg RxMessage = {0};
 	FDCAN_RxHeaderTypeDef RxHeader = {0};
	uint8_t RxData[8] = {0};	
	Message rxm = {0};
//CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
	// Drop extended frames
//if(RxMessage.IDE == CAN_ID_EXT) //不处理扩展帧
	if(RxHeader.Identifier > 0x7FF) //不处理扩展帧	
			return;
//rxm.cob_id = RxMessage.StdId;
	rxm.cob_id = RxHeader.Identifier;
// 	if(RxMessage.RTR == CAN_RTR_REMOTE)//远程帧
  	if(RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)//远程帧	
	rxm.rtr = 1;
//rxm.len = RxMessage.DLC;
//rxm.len = RxHeader.DataLength;
		switch(RxHeader.DataLength){
			case FDCAN_DLC_BYTES_0:
				rxm.len = 0;
				break;
			case FDCAN_DLC_BYTES_1:
				rxm.len = 1;
				break;
			case FDCAN_DLC_BYTES_2:
				rxm.len = 2;
				break;
			case FDCAN_DLC_BYTES_3:
				rxm.len = 3;
				break;
			case FDCAN_DLC_BYTES_4:
				rxm.len = 4;
				break;
			case FDCAN_DLC_BYTES_5:
				rxm.len = 5;
				break;
			case FDCAN_DLC_BYTES_6:
				rxm.len = 6;
				break;
			case FDCAN_DLC_BYTES_7:
				rxm.len = 7;
				break;
			case FDCAN_DLC_BYTES_8:
				rxm.len = 8;
				break;
		}
		
	for(i=0 ; i<rxm.len ; i++)
		{
//		 rxm.data[i] = RxMessage.Data[i];
  		 rxm.data[i] = RxData[i];		
//			printf("rxm.data[%d] = %d\r\n",i,rxm.data[i]);		//printf,whu
		}
		
		/*****这里是自己的数据接收部分*******/
	if( rxm.cob_id>>7==0xB)	//快速SDO回应ID为0x580+对方id, 右移7位即为0xB 
	{
		int16_t test=0;
		switch(rxm.cob_id)
		{
			case 0x582:
				test=rxm.data[4]|rxm.data[5]<<8;
//				printf("test=%d,\r\n",test);	//其实在中断不应该使用打印函数的，这里是方便调试而已,whu,remove??
				break;
			default:
				break;
		}
	
	}
	else
//		;
		canDispatch(co_data, &rxm);	
	//CANopen自身的处理函数，因为快速SDO不需要反馈，所以在上边处理后就不需要调用这步了
		
  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
//void TIM3_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM3_IRQn 0 */

//  /* USER CODE END TIM3_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim3);
//  /* USER CODE BEGIN TIM3_IRQn 1 */
//////	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
//////	{
//////		Error_Handler();
//////	}

//////	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

//////	++TxData[0];

//////	HAL_Delay(1);
////	
//  /* USER CODE END TIM3_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
