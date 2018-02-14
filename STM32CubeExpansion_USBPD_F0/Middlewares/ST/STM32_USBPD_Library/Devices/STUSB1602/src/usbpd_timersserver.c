/**
  ******************************************************************************
  * @file    usbpd_timersserver.c
  * @author  MCD Application Team
  * @brief   This file contains timer server functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
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
#if defined(STM32F334x8)
#define TIMx                           TIM17
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM17_CLK_ENABLE
#define TIMx_IRQ                       TIM17_IRQn
#else
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM2_CLK_ENABLE
#define TIMx_IRQ                       TIM2_IRQn
#endif

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
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM2 counter clock equal to 1 MHz */
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000));

  /* Set the auto-reload value to have a counter frequency of 5 kHz */
  LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM2), 500));
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output compare mode: TOGGLE */ 
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_TOGGLE);

  /* Set output channel polarity: OC is active high */
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM2);
}

void USBPD_TIM_Start(TIM_identifier id, uint16_t us_time)
{
    /* Positionne l'evenement pour sa detection */
    switch(id)
    {
    case TIM_PORT0_CRC:
      LL_TIM_OC_SetCompareCH1(TIM2, (us_time + TIM2->CNT)%2000);
      LL_TIM_ClearFlag_CC1(TIM2);     
    break;
    case TIM_PORT0_RETRY:
      LL_TIM_OC_SetCompareCH2(TIM2, (us_time + TIM2->CNT)%2000);
      LL_TIM_ClearFlag_CC2(TIM2);
    break;
    case TIM_PORT1_CRC:
      LL_TIM_OC_SetCompareCH3(TIM2, (us_time + TIM2->CNT)%2000);
      LL_TIM_ClearFlag_CC3(TIM2);
    break;
    case TIM_PORT1_RETRY:
      LL_TIM_OC_SetCompareCH4(TIM2, (us_time + TIM2->CNT)%2000);
      LL_TIM_ClearFlag_CC4(TIM2);
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
    return LL_TIM_IsActiveFlag_CC1(TIM2);
  case TIM_PORT0_RETRY:
    return LL_TIM_IsActiveFlag_CC2(TIM2);
  case TIM_PORT1_CRC:
    return LL_TIM_IsActiveFlag_CC3(TIM2);
  case TIM_PORT1_RETRY:
    return LL_TIM_IsActiveFlag_CC4(TIM2);
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

