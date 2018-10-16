/**
  ******************************************************************************
  * @file    usbpd_hw_if_it.c
  * @author  System Lab
  * @brief   This file contains HW interface interrupt routines.
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
#include "usbpd_hw_if.h"
#if !defined(USE_HAL_SPI)
#include "stm32f0xx_ll_dma.h"
#endif /* USE_HAL_SPI */
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];
#if defined(USE_HAL_ADC)
extern ADC_HandleTypeDef        usbpdm1_hadc;
#endif /* USE_HAL_ADC */

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE void DMA_IRQHandler(uint8_t PortNum);
__STATIC_INLINE void RX_TIM_Interrupt_IRQHandler(uint8_t PortNum);
__STATIC_INLINE void RX_COUNTTIM_IRQHandler(uint8_t PortNum);
/* Private functions ---------------------------------------------------------*/
/* Optimized functions */
#if defined(USE_HAL_TIM)
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit);
#else
static inline void SINGLE_TIM_IRQHandler(uint8_t PortNum);
#endif /* USE_HAL_TIM */

/**
  * @brief This function handles DMA1 channel 4, 5, 6 and 7 interrupts.
  */
void USBPD_DMA_PORT0_IRQHandler(void)
{
  DMA_IRQHandler(0);
}

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void USBPD_DMA_PORT1_IRQHandler(void)
{
  DMA_IRQHandler(1);
}

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  * @param  PortNum The current port number
  */
__STATIC_INLINE void DMA_IRQHandler(uint8_t PortNum)
{
  uint8_t *end;

#if defined(USE_HAL_SPI)
  /* Handler DMA TX PORT  */
  if (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
#else
   /* LL DMA TX PORT  */
  if (LL_DMACH_GET_FLAG_TC(PortNum, TX_DMA(PortNum)) != RESET)   
#endif /* USE_HAL_SPI */
  {
    if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BIST)
    {
      Ports[PortNum].BIST_index += 1;
      if (Ports[PortNum].BIST_index == BIST_MAX_LENGTH)
      {
        /* Get the address of the transmission buffer*/
        end = Ports[PortNum].pTxBuffPtr;
        end[(TX_BUFFER_LEN) * 4 - 1] = 0;
        USBPDM1_Set_DMA_Normal_Mode(PortNum);
      }
      if (Ports[PortNum].BIST_index > BIST_MAX_LENGTH)
      {
        USBPDM1_TX_Done(PortNum);
        Ports[PortNum].BIST_index = 0;
      }
    }
    /* Transfer complete interrupt is used to end the transmission */
    else
    {
      USBPDM1_TX_Done(PortNum);
      if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
      {
        Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum);
      }
    }
  }
#if defined(USE_HAL_SPI)
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmatx));
#else
  LL_DMACH_CLEAR_FLAG_GI(PortNum, TX_DMA(PortNum));
#endif /* USE_HAL_SPI */
}

/**
  * @brief This function handles RX_TIM global interrupt for Port 0.
  */
void USBPD_RX_PORT0_Interrupt_IRQHandler(void)
{
  RX_TIM_Interrupt_IRQHandler(0);
}

/**
  * @brief This function handles RX_TIM global interrupt for Port 1.
  */
void USBPD_RX_PORT1_Interrupt_IRQHandler(void)
{
  RX_TIM_Interrupt_IRQHandler(1);
}

/**
  * @brief This function handles RX_TIM global interrupt generic port.
  * @param  PortNum The current port number
  */
#if defined(_OPTIM_CONSO)
extern   void SystemClock_Config_48Mhz(void);
extern volatile uint32_t FlagExplicitContract;
#endif /* _OPTIM_CONSO */

__STATIC_INLINE void RX_TIM_Interrupt_IRQHandler(uint8_t PortNum)
{
#if defined(_OPTIM_CONSO)
  SystemClock_Config_48Mhz();
  FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */

  /* PC04 A8 set : Rx start */
  GPIOA->BSRR = GPIO_PIN_8;

  /* This interrupt is used just to detect the first edge */
#if defined(USE_HAL_TIM)
  USBPD_SINGLE_TIM_IC_Stop_IT(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum), RX_TIMCH_TIMIT(PortNum));
  __HAL_TIM_CLEAR_IT(&(Ports[PortNum].htimrx), RX_TIMCH_TIMIT(PortNum));
#else
  USBPD_SINGLE_TIM_IC_Stop_IT(PortNum, RX_TIMCH(PortNum));
  LL_TIM_ClearFlag_CC1(RX_TIM(PortNum));
#endif /* USE_HAL_TIM */

  /* Stop the TIM DMA transfers */
#if defined(USE_HAL_TIM)
  HAL_TIM_IC_Stop_DMA(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
#else
  LL_TIM_DisableDMAReq_CC1(RX_TIM(PortNum));
  LL_TIM_CC_DisableChannel(RX_TIM(PortNum), RX_TIMCH(PortNum));
  LL_TIM_DisableCounter(RX_TIM(PortNum));
#endif

  /* DMA Abort to execute the DMA Deinit */
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);

#if defined(USE_HAL_TIM)
  Ports[PortNum].htimrx.Instance->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP; //BOTHEDGE
#else
  LL_TIM_IC_SetPolarity(RX_TIM(PortNum), RX_TIMCH(PortNum), LL_TIM_IC_POLARITY_BOTHEDGE);
#endif /* USE_HAL_TIM */

  /* Variables for decoding stage are initalized */
#if defined(USE_HAL_TIM)
  (Ports[PortNum].htimrx).Instance->CNT = 0;
#else
  LL_TIM_SetCounter(RX_TIM(PortNum), 0);
#endif
  RX_Init_Hvar(PortNum);
  Ports[PortNum].pRxDataPtr[0] = 0;

  /* The timer is configured to capture data with DMA transfer.
   * The number of DMA transfers is set to the maximum possible
  */
#if defined(USE_HAL_TIM)
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  __HAL_TIM_ENABLE_DMA(&(Ports[PortNum].htimrx), RX_TIM_DMA_CC(PortNum));
  HAL_TIM_IC_Start(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
#else
  HAL_DMA_Start(&Ports[PortNum].hdmarx, (uint32_t)&((RX_TIM(PortNum))->CCR1), (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  LL_TIM_EnableDMAReq_CC1(RX_TIM(PortNum));
  LL_TIM_CC_EnableChannel(RX_TIM(PortNum), RX_TIMCH(PortNum));
  LL_TIM_EnableCounter(RX_TIM(PortNum));
#endif /* USE_HAL_TIM */

  /* The auxiliary timer is started */
#if defined(USE_HAL_TIM)
  (Ports[PortNum].htimcountrx).Instance->CNT = 11;
  HAL_TIM_OC_Start_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum));
#else
  LL_TIM_SetCounter(RX_COUNTTIM(PortNum), 11);
  LL_TIM_EnableIT_CC1(RX_COUNTTIM(PortNum));
  LL_TIM_CC_EnableChannel(RX_COUNTTIM(PortNum), RX_COUNTTIMCH(PortNum));
  LL_TIM_EnableCounter(RX_COUNTTIM(PortNum));
#endif /* USE_HAL_TIM */
}

/**
  * @brief This function handles COUNT_TIM global interrupt for Port 0.
  */
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void)
{
  RX_COUNTTIM_IRQHandler(0);
}

/**
  * @brief This function handles COUNT_TIM global interrupt for Port 1.
  */
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void)
{
  RX_COUNTTIM_IRQHandler(1);
}

/**
  * @brief This function handles COUNT_TIM global interrupt.
  * @param  PortNum The current port number
  */
__STATIC_INLINE void RX_COUNTTIM_IRQHandler(uint8_t PortNum)
{
  /* Actions are managed by callbacks */
#if defined(USE_HAL_TIM)
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum));
#else
  SINGLE_TIM_IRQHandler(PortNum);
#endif /* USE_HAL_TIM */
}

/**
  * @brief This function handles ADC interrupts
  */
void ADC1_COMP_IRQHandler(void)
{
#if defined(USE_HAL_ADC)
  /* The AWD callback is called */
  HAL_ADC_IRQHandler(&usbpdm1_hadc);
#else
#if (0) /* ADC AWD IT not used */
  /* Check whether ADC analog watchdog 1 caused the ADC interruption */
  if(LL_ADC_IsActiveFlag_AWD1(P_NUCLEO_USB001_ADC) != 0)
  {
    /* Clear flag ADC analog watchdog 1 */
    LL_ADC_ClearFlag_AWD1(P_NUCLEO_USB001_ADC);
    
    /* Call interruption treatment function */
    ..._Callback();
  }
#endif
#endif /* USE_HAL_ADC */
}

/* -------------------------------------------------------------------------- */
/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
  * @brief  This function handles TIM interrupts requests.
  * @param  htim  TIM  handle
  * @param  Flag  TIM  flg
  * @param  Timit TIM  IT
  * @retval None
  */
#if defined(USE_HAL_TIM)
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit)
{
  /* Capture compare event */
  if (__HAL_TIM_GET_FLAG(htim, Flag) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, Timit) != RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(htim, Timit);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
        /* Output compare event */
        HAL_TIM_OC_DelayElapsedCallback(htim);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
      }
    }
  }
  /* TIM Update event */
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
      HAL_TIM_PeriodElapsedCallback(htim);
    }
  }
}
#else
static inline void SINGLE_TIM_IRQHandler(uint8_t PortNum)
{
  /* Capture compare event */
  if (LL_TIM_IsActiveFlag_CC1(RX_COUNTTIM(PortNum)) != RESET)
  {
    if (LL_TIM_IsEnabledIT_CC1(RX_COUNTTIM(PortNum)) != RESET)
    {
      {
        LL_TIM_ClearFlag_CC1(RX_COUNTTIM(PortNum));
        /* Output compare event */
        DelayElapsedCallback(PortNum);
      }
    }
  }
  
  /* TIM Update event */
  if (LL_TIM_IsActiveFlag_UPDATE(RX_COUNTTIM(PortNum)) != RESET)
  {
    if (LL_TIM_IsEnabledIT_UPDATE(RX_COUNTTIM(PortNum)) != RESET)
    {
      LL_TIM_ClearFlag_UPDATE(RX_COUNTTIM(PortNum));
    }
  }
}
#endif /* USE_HAL_TIM */

/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

