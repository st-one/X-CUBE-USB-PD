/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    29-June-2016
  * @brief   Default Interrupt Service Routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  osSystickHandler();
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

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief  This function handles EXTI line 4 to 15 interrupts.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief  This function handles DMA Channel 2 and 3 interrupts.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  USBPD_DMA_PORT1_IRQHandler();
}

/**
  * @brief  This function handles DMA Channel 4 to 7 interrupts.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  USBPD_DMA_PORT0_IRQHandler();
}

/**
  * @brief  This function handles TIM1 CC interrupt..
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  USBPD_RX_PORT1_Interrupt_IRQHandler();
}

/**
  * @brief  This function handles TIM3 interrupt.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  USBPD_RX_PORT0_Interrupt_IRQHandler();
}

/**
  * @brief  This function handles TIM6 interrupt.
  * @param  None
  * @retval None
  */
void TIM16_IRQHandler(void)
{
  USBPD_RX_PORT0_COUNTTIM_IRQHandler();
}

/**
  * @brief  This function handles TIM7 interrupt.
  * @param  None
  * @retval None
  */
void TIM17_IRQHandler(void)
{
  USBPD_RX_PORT1_COUNTTIM_IRQHandler();
}

/**
  * @brief  This function handles TIM2 interrupt.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  USBPD_PE_PRL_TIMER_IRQHandler();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


