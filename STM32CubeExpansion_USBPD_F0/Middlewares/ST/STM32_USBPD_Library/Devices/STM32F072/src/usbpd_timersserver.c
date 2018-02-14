/**
  ******************************************************************************
  * @file    usbpd_timersserver.c
  * @author  MCD Application Team
  * @brief   This file contains timer server functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_timersserver.h"
#include "stm32f0xx_ll_tim.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_TIMESERVER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Definition for TIMx clock resources */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM2_CLK_ENABLE
#define TIMx_IRQ                       TIM2_IRQn

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize Timer 2
  * @retval None
  */
void USBPD_TIM_Init(void)
{
  TIMx_CLK_ENABLE();
  /***************************/
  /* Time base configuration */
  /***************************/
  /* Counter mode: select up-counting mode */
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIMx counter clock equal to 1 MHz */
  LL_TIM_SetPrescaler(TIMx, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000));

  /* Set the auto-reload value to have a counter frequency of 5 kHz */
  LL_TIM_SetAutoReload(TIMx, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIMx), 500));
  
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

void USBPD_TIM_Start(TIM_identifier id, uint16_t us_time)
{
    /* Positionne l'evenement pour sa detection */
    switch(id)
    {
    case TIM_PORT0_CRC:
      LL_TIM_OC_SetCompareCH1(TIMx, (us_time + TIMx->CNT)%2000);
      LL_TIM_ClearFlag_CC1(TIMx);     
      break;
    case TIM_PORT0_RETRY:
      LL_TIM_OC_SetCompareCH2(TIMx, (us_time + TIMx->CNT)%2000);
      LL_TIM_ClearFlag_CC2(TIMx);
      break;
    case TIM_PORT1_CRC:
      LL_TIM_OC_SetCompareCH3(TIMx, (us_time + TIMx->CNT)%2000);
      LL_TIM_ClearFlag_CC3(TIMx);
      break;
    case TIM_PORT1_RETRY:
      LL_TIM_OC_SetCompareCH4(TIMx, (us_time + TIMx->CNT)%2000);
      LL_TIM_ClearFlag_CC4(TIMx);
      break;
    default:
      break;
   }
}

uint8_t USBPD_TIM_IsExpired(TIM_identifier id)
{
  switch(id)
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

