/**
  ******************************************************************************
  * @file    usbpd_timersserver.c
  * @author  MCD Application Team
  * @brief   This file contains timer server functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "usbpd_timersserver.h"
#include "stm32f0xx_ll_tim.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_TIMESERVER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Definition for TIMx clock resources */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMx_IRQ                       TIM2_IRQn

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initialization of TIMERSERVER, used for CRC and RETRY operations
  * @retval None
  */
void USBPD_TIM_Init(void)
{
  TIMx_CLK_ENABLE;

  /***************************/
  /* Time base configuration */
  /***************************/

  /* Counter mode: select up-counting mode */
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIMx counter clock equal to 1 MHz */
  LL_TIM_SetPrescaler(TIMx, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000));

  /* Set the auto-reload value to have a counter frequency of 250Hz */
  LL_TIM_SetAutoReload(TIMx, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIMx), 250));

  /*********************************/
  /* Output waveform configuration */
  /*********************************/

  /* Set output compare mode: TOGGLE */
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_TOGGLE);

  /* Set output channel polarity: OC is active high */
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);

  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
}


/**
  * @brief  Start TIMERSERVER (CRC and RETRY operation)
  * @param  id Timer Operation Identifier
  * @param  us_time time in micro-seconds
  * @retval None
  */
void USBPD_TIM_Start(TIM_identifier id, uint16_t us_time)
{
  switch (id)
  {
    case TIM_PORT0_CRC:
      LL_TIM_OC_SetCompareCH1(TIMx, (us_time + TIMx->CNT) % 4000);
      LL_TIM_ClearFlag_CC1(TIMx);
      break;
    case TIM_PORT0_RETRY:
      LL_TIM_OC_SetCompareCH2(TIMx, (us_time + TIMx->CNT) % 4000);
      LL_TIM_ClearFlag_CC2(TIMx);
      break;
    case TIM_PORT1_CRC:
      LL_TIM_OC_SetCompareCH3(TIMx, (us_time + TIMx->CNT) % 4000);
      LL_TIM_ClearFlag_CC3(TIMx);
      break;
    case TIM_PORT1_RETRY:
      LL_TIM_OC_SetCompareCH4(TIMx, (us_time + TIMx->CNT) % 4000);
      LL_TIM_ClearFlag_CC4(TIMx);
      break;
    default:
      break;
  }
}


/**
  * @brief  Retrieve the TIMERSERVER status for a specified id
  * @param  id Timer Operation Identifier
  * @retval State of bit (1 or 0).
  */
uint8_t USBPD_TIM_IsExpired(TIM_identifier id)
{
  switch (id)
  {
    case TIM_PORT0_CRC:
      return LL_TIM_IsActiveFlag_CC1(TIMx);
    case TIM_PORT0_RETRY:
      return LL_TIM_IsActiveFlag_CC2(TIMx);
    case TIM_PORT1_CRC:
      return LL_TIM_IsActiveFlag_CC3(TIMx);
    case TIM_PORT1_RETRY:
      return LL_TIM_IsActiveFlag_CC4(TIMx);
    default:
      break;
  }
  return 1;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

