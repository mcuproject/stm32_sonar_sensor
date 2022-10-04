
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

uint32_t	tick_count; // sys_tick
uint32_t	tick_flag;

#define micros() TIM5->CNT
void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}
	  
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  tick_flag = 1;
  tick_count++;
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;

void TIM3_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM3);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value;

    /* Frequency computation 
       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

    Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
}

uint16_t pb0_count;
uint32_t last_pb0_cnt;
uint32_t pulse_width;
uint8_t int_status;
uint8_t distance_request;
void EXTI0_IRQHandler(void)
{
	pb0_count++;
	int_status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == SET)
  {
		last_pb0_cnt = micros();  
  }
  else
  {
		pulse_width = micros()-last_pb0_cnt;
  }
	if (pulse_width >= 8 || pulse_width <= 15) {
		distance_request = 1;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		TIM_Cmd(TIM4, ENABLE);
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

uint32_t tim4_int;
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
		tim4_int++;
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //update interrupt is set by the hardware, and it's cleared by the software.
		TIM_Cmd(TIM4, DISABLE);
  }
}

