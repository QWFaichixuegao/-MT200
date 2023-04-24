/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "can_device.h"
#include "cmsis_os.h"
#include "canfestival.h"

uint8_t  g_CAN1_RxData[8];    /* CAN���߽��ջ��� */
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
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

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void can_filter_config(void)
{
	/*##-2- Configure the CAN Filter ###########################################*/
	CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;//��FilterNumber������0 �������0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//��������ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//����32λ����ģʽ
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//ɸѡ������CAN_FILTER_FIFO0
  sFilterConfig.FilterIdHigh = 0x0000; //���ù�����ID��16λ
  sFilterConfig.FilterIdLow = 0x0000;//���ù�����ID��16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;//���ù����������16λ
  sFilterConfig.FilterMaskIdLow = 0x0000;//���ù����������16λ
//  sFilterConfig.SlaveStartFilterBank = 14;//�� BankNumber �������ֻ��CAN2����
	sFilterConfig.FilterActivation = ENABLE;//�򿪹�����
  if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig) != HAL_OK)//��ʼ��������
  {
   Error_Handler();
  }
	/*##-3- Start the CAN peripheral ###########################################*/
	if(HAL_CAN_Start(&hcan1) != HAL_OK)//��can
  {
   Error_Handler();
  }
	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
	
}
 
/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
uint8_t testda[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
//CANopen����
//  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_handle.RxHeader, can_handle.RxData) != HAL_OK)
//  {
//    /* Reception Error */
//    Error_Handler();
//  }
  /* Display LEDx */
	//moto
//  if (can_handle.RxHeader.StdId == MOTO_ReadACK_ID)
//  {
//		moto_ack_handle();
//  }
//	//battery
//  else if (can_handle.RxHeader.StdId == BAT_DATA_StdId)
//  {
//		battery_ack_handle();
//  }
	//driver&sensor
//	driver_sensor_ack_handle();
//CANopen����	
		if(hcan->Instance==CAN1)
		{
			//CAN_RxHeaderTypeDef   RxHeader;                                     /* ���վ�� */	
			HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &can_handle.RxHeader,can_handle.RxData); /* �����ݴ�������ȡ�� */
			  if (can_handle.RxHeader.StdId == MOTO_ReadACK_ID)
				{
						moto_ack_handle();
				}
				else if (can_handle.RxHeader.StdId == BAT_DATA_StdId)
				{
						battery_ack_handle();
				}
				else
				{
						canReceive_Callback(&can_handle.RxHeader,can_handle.RxData);
				}
			
		}
}

void send_moto_cmd(uint8_t address, uint8_t regis_num, uint16_t variable, uint16_t MOTO_ID)
{
	
	can_handle.TxData[0] = address;						//���ָ��Ĵ�����ַ	
	can_handle.TxData[1] = regis_num;							//�����Ĵ�����	
	can_handle.TxData[2] = variable&0xff;			//�ٶ�ֵ��8λ��ǰ
	can_handle.TxData[3] = (variable>>8)&0xff;
	can_handle.TxData[4] = 0x00;
	can_handle.TxData[5] = 0x00;
	can_handle.TxData[6] = 0x00;
	can_handle.TxData[7] = 0x00;
		
  can_handle.TxHeader.StdId = MOTO_ID;
	can_handle.TxHeader.DLC   = MOTO_DLC;
	can_handle.TxHeader.IDE		= CAN_ID_STD;
	can_handle.TxHeader.RTR		= CAN_RTR_DATA;
	can_handle.TxHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(&hcan1, &can_handle.TxHeader, can_handle.TxData,&can_handle.TxMailbox);
	vTaskDelay(2);	
}


void send_driver_cmd( uint8_t driver, uint16_t variable, uint32_t stdid)
{
	switch(driver)
		{
			case LED48V://���
				can_handle.TxData[2] = LED48V;
				can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0 and 100
				
			break;
			
			case BOXFAN:
				can_handle.TxData[2] = BOXFAN;
				can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	100
				
			break;
			
			case CARFAN:
				can_handle.TxData[2] = CARFAN;
			  can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	100
				
			break;
			
			case DRAUGHT://���
				can_handle.TxData[2] = DRAUGHT;
			  can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	100
//				can_handle.TxData[4] = ((variable>>8)&0xff);
			break;
			
			case PUMP://ˮ��
				can_handle.TxData[2] = PUMP;
				can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	100
			break;	
			
			case LED_SER://��������
				can_handle.TxData[2] = LED_SER;
				can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	255
			break;
			case SPEECH://����ģ��
				can_handle.TxData[2] = SPEECH;
				can_handle.TxData[4] = 0x00;
				can_handle.TxData[5] = (variable&0xff);//0	 -	255
			break;				
		}
	can_handle.TxData[0] = 0xaa;	
	can_handle.TxData[1] = 0x03;	//����ָ���0x03
	can_handle.TxData[3] = 0x02;	//����ָ���0x02
	can_handle.TxData[6] = 0x55;
	can_handle.TxData[7] = 0xaa;
	
  can_handle.TxHeader.StdId	=	stdid;
	can_handle.TxHeader.DLC		=	DATADLC;//����֡���ݶεĳ���
	can_handle.TxHeader.IDE		=	CAN_ID_STD;//ʹ�ñ�׼֡
	can_handle.TxHeader.RTR		=	CAN_RTR_DATA;//����������֡
	can_handle.TxHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(&hcan1, &can_handle.TxHeader, can_handle.TxData, &can_handle.TxMailbox);
	vTaskDelay(2);
}


void check_driver_sensor(uint8_t driver, uint32_t stdid)
{
	switch(driver)
		{
			case SCRAM:
				can_handle.TxData[2] = SCRAM;
				break;
			case TEMP:
				can_handle.TxData[2] = TEMP;
				break;
			case ANGLE:
				can_handle.TxData[2] = ANGLE;
				break;	
			case HUMITURE1:
				can_handle.TxData[2] = HUMITURE1;
				break;	
			case HUMITURE2:
				can_handle.TxData[2] = HUMITURE2;
				break;				
			case ALLDEV:
				can_handle.TxData[2] = ALLDEV;
				break;
		}

	can_handle.TxData[0] = 0xaa;	
	can_handle.TxData[1] = 0x02;	//��ȡ���ȶ���0x02
	can_handle.TxData[3] = 0x01;	//��ȡָ���0x01
	can_handle.TxData[4] = 0x00;	//��ȡָ���0x00
	can_handle.TxData[5] = 0x00;	//��ȡָ���0x00
	can_handle.TxData[6] = 0x55;
	can_handle.TxData[7] = 0xaa;	
	
	can_handle.TxHeader.StdId	=	stdid;
	can_handle.TxHeader.DLC		=	DATADLC;//����֡���ݶεĳ���
	can_handle.TxHeader.IDE		=	CAN_ID_STD;//ʹ�ñ�׼֡
	can_handle.TxHeader.RTR		=	CAN_RTR_DATA;//����������֡
	can_handle.TxHeader.TransmitGlobalTime = DISABLE;
		
	HAL_CAN_AddTxMessage(&hcan1, &can_handle.TxHeader, can_handle.TxData, &can_handle.TxMailbox);
	vTaskDelay(2);		
}
/* USER CODE END 1 */
