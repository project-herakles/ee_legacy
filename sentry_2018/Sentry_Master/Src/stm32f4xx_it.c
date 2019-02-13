/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
#include "tim.h"
#include "testTask.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal.h"
#include "controlTask.h"
#include "remoteTask.h"
#include "can.h"
#include "COM_Task.h"
#include "usart.h"
#include "dma_hku.h"

extern uint8_t RemoteBuffer[18];
extern uint8_t *visionBuffer;
extern uint8_t canRxMsg[8];
extern uint8_t canTxMsg[8];
extern CAN_TxHeaderTypeDef can1TxHeader0;
extern CAN_TxHeaderTypeDef can1TxHeader1;
extern CAN_RxHeaderTypeDef can1RxHeader;

uint8_t rx_buffer[256];
uint8_t counter = 0;
uint8_t index_r = 0;
uint8_t data = 0;
int16_t ch0;
Judge_RxState_e JG_RxState;
Judge_RxStruct JG_RxStruct;
static uint8_t data_cnt = 0;
int8_t debug_DMA_flag = 1;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&can1RxHeader,canRxMsg);
	CanReceiveMsgProcess(&can1RxHeader,canRxMsg);
	//CanReceiveMsgProcess(&can1RxHeader,canRxMsg);
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uart_receive_handler(&huart1);

	//RED_LED_ON();
	/*
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET)
	{
		RemoteBuffer[index_r++] = huart1.Instance->DR;
	}
	if(index_r==18)
	{
		remoteDataProcess(RemoteBuffer);
		index_r = 0;
		//COM_Upload_PC();
	}
	*/
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
static uint8_t byte_s = 0;
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint8_t stateflag = READ_REG(huart3.Instance->SR);
	if((stateflag & USART_SR_RXNE) != RESET)
	{
		uint8_t inByte = READ_REG(huart3.Instance->DR);
		byte_s = inByte;
		switch(JG_RxState)
		{
			case SOF:
			{
				if(inByte==0xA5) JG_RxState = LENGTH_LOW;
			}break;
			case LENGTH_LOW:
			{
				JG_RxStruct.Length = inByte;
				JG_RxState = LENGTH_HIGH;
			}break;
			case LENGTH_HIGH:
			{
				JG_RxStruct.Length |= (inByte<<8);
				if(JG_RxStruct.Length<=20) JG_RxState = SEQ;
				else JudgeRx_Reset(&JG_RxState,&JG_RxStruct);
				
			}break;
			case SEQ:
			{
				JG_RxStruct.Seq = inByte;
				JG_RxState = CRC8;
			}break;
			case CRC8:
			{
				JG_RxStruct.CRC8 = inByte;
				if(CRC8_check(&JG_RxStruct)==1) JG_RxState = CMD_LOW;
				else JudgeRx_Reset(&JG_RxState,&JG_RxStruct);
			}break;
			case CMD_LOW:
			{
				JG_RxStruct.Cmd = inByte;
				JG_RxState = CMD_HIGH;
			}break;
			case CMD_HIGH:
			{
				JG_RxStruct.Cmd |= (inByte<<8);
				if(JG_RxStruct.Cmd == 0x02) JG_RxState = DATA; //receive hurt info only
				else JudgeRx_Reset(&JG_RxState,&JG_RxStruct);
			}break;
			case DATA:
			{
				if(data_cnt<JG_RxStruct.Length)
					JG_RxStruct.Data[data_cnt++] = inByte;
				else
				{
					JG_RxState = CRC16_LOW;
					data_cnt = 0;//reset data counter
				}
			}break;
			case CRC16_LOW:
			{
				JG_RxStruct.CRC16 = inByte;
				JG_RxState = CRC16_HIGH;
			}break;
			case CRC16_HIGH:
			{
				JG_RxStruct.CRC16 |= (inByte<<8);
				if(CRC16_check(&JG_RxStruct)) //COM_Judge_Process(&JG_RxStruct);//if CRC16 checked, process data
				JudgeRx_Reset(&JG_RxState,&JG_RxStruct);//reset rxStruct and rxState
			}break;
			default:
			{
				JudgeRx_Reset(&JG_RxState,&JG_RxStruct);
			}
		}
	}
  /* USER CODE END USART3_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	uint8_t stateflag = READ_REG(huart4.Instance->SR);
	if((stateflag & USART_SR_RXNE) != RESET)
	{
		RED_LED_ON();
		uint8_t inByte = READ_REG(huart4.Instance->DR);
		COM_Receive_PC_handler(inByte);
	}
	
  /* USER CODE END UART4_IRQn 0 */
  //HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	//RED_LED_TOGGLE();
	Control_Loop();//called every 1ms
	//Test_Task();
	
	GREEN_LED_TOGGLE();
	//RED_LED_OFF();
	
	
	
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//RED_LED_ON();
	/*
	ch0 = ((uint16_t)RemoteBuffer[0] | ((uint16_t)RemoteBuffer[1] << 8)) & 0x07FF;
	uint16_t power = ch0 - 1024;
	canTxMsg[4] = (uint8_t)power>>8;
	canTxMsg[5] = (uint8_t)power;
	*/
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
	debug_DMA_flag++;
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ /*
	RED_LED_ON();
	ch0 = ((int16_t)RemoteBuffer[0] | ((int16_t)RemoteBuffer[1] << 8)) & 0x07FF;
	power = (ch0 - 1024)*20;
	canTxMsg[4] = (uint8_t)(power >> 8);
	canTxMsg[5] = (uint8_t) power;
	HAL_UART_Receive_IT(&huart1,RemoteBuffer,18);
	*/
	
	//remoteDataProcess(RemoteBuffer);
	//HAL_UART_Receive_IT(&huart1,RemoteBuffer,18);
	HAL_UART_Receive_DMA(&huart1,RemoteBuffer,18);//??
	
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  return;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
