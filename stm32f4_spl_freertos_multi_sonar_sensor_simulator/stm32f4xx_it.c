/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
// void SVC_Handler(void)
// {
// }

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
// void PendSV_Handler(void)
// {
// }

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// void SysTick_Handler(void)
// {
// }

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
}

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
}

#define micros() TIM5->CNT

uint32_t pulse_width0, pb0_count;
uint8_t sonar0_request;
void EXTI0_IRQHandler(void)
{
	static uint32_t last_pb0_cnt;
	pb0_count++;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == SET)
  {
		last_pb0_cnt = micros();  
  }
  else
  {
		pulse_width0 = micros()-last_pb0_cnt;
  }
	if (pulse_width0 >= 8 || pulse_width0 <= 15) {
		sonar0_request = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

uint16_t pb1_count;
uint32_t pulse_width1;
uint8_t sonar1_request;
void EXTI1_IRQHandler(void)
{
	static uint32_t last_pb1_cnt;
	pb1_count++;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == SET)
  {
		last_pb1_cnt = micros();  
  }
  else
  {
		pulse_width1 = micros()-last_pb1_cnt;
  }
	if (pulse_width1 >= 8 || pulse_width1 <= 15) {
		sonar1_request = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}

uint16_t pb2_count;
uint8_t sonar2_request, pulse_width2;
void EXTI2_IRQHandler(void)
{
	static uint32_t last_cnt;
	pb2_count++;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == SET)
  {
		last_cnt = micros();  
  }
  else
  {
		pulse_width2 = micros()-last_cnt;
  }
	if (pulse_width2 >= 8 || pulse_width2 <= 15) {
		sonar2_request = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}


uint16_t pb3_count;
uint8_t sonar3_request, pulse_width3;
void EXTI3_IRQHandler(void)
{
	static uint32_t last_cnt;
	pb3_count++;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == SET)
  {
		last_cnt = micros();  
  }
  else
  {
		pulse_width3 = micros()-last_cnt;
  }
	if (pulse_width3 >= 8 || pulse_width3 <= 15) {
		sonar3_request = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}


uint16_t pb4_count;
uint8_t sonar4_request, pulse_width4;
void EXTI4_IRQHandler(void)
{
	static uint32_t last_cnt;
	pb4_count++;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == SET)
  {
		last_cnt = micros();  
  }
  else
  {
		pulse_width4 = micros()-last_cnt;
  }
	if (pulse_width4 >= 8 || pulse_width4 <= 15) {
		sonar4_request = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
