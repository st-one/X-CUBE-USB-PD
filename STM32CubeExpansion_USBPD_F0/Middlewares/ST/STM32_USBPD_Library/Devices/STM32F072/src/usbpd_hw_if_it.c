/**
  ******************************************************************************
  * @file    usbpd_hw_if_it.c
  * @author  System Lab
  * @brief   This file contains HW interface interrupt routines.
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
#include "usbpd_hw_if.h"
#include "usbpd_cad.h"
#include "string.h"
//#include "stm32f0xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];
extern ADC_HandleTypeDef        usbpdm1_hadc;

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE void DMA_IRQHandler(uint8_t PortNum);
__STATIC_INLINE void RX_TIM_Interrupt_IRQHandler(uint8_t PortNum);
__STATIC_INLINE void RX_COUNTTIM_IRQHandler(uint8_t PortNum);
/* Private functions ---------------------------------------------------------*/
/* Optimized functions */
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit);

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
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
  DMA_IRQHandler(0);
#else
  DMA_IRQHandler(1);
#endif
}

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  * @param  PortNum The current port number
  */
__STATIC_INLINE void DMA_IRQHandler(uint8_t PortNum)
{
  uint8_t *end;

  /* Handler DMA TX PORT  */
  if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
  {
    if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
    { 
      Ports[PortNum].BIST_index+=1;
      if(Ports[PortNum].BIST_index == BIST_MAX_LENGTH) 
      {
        /* Get the address of the transmission buffer*/
        end = Ports[PortNum].pTxBuffPtr;
        end[(TX_BUFFER_LEN)*4 - 1] = 0;
        USBPDM1_Set_DMA_Normal_Mode(PortNum); 
      }
      if(Ports[PortNum].BIST_index > BIST_MAX_LENGTH) 
      {
        USBPDM1_TX_Done(PortNum);
        Ports[PortNum].BIST_index=0;
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
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmatx));
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
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
  RX_TIM_Interrupt_IRQHandler(0);  	
#else
  RX_TIM_Interrupt_IRQHandler(1);
#endif
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
  USBPD_SINGLE_TIM_IC_Stop_IT(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum), RX_TIMCH_TIMIT(PortNum));
  __HAL_TIM_CLEAR_IT(&(Ports[PortNum].htimrx), RX_TIMCH_TIMIT(PortNum));

  /* Stop the TIM DMA transfers */
  HAL_TIM_IC_Stop_DMA(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
    
  /* DMA Abort to execute the DMA Deinit */
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
    
  Ports[PortNum].htimrx.Instance->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP; //BOTHEDGE
  
  /* Variables for decoding stage are initalized */
  (Ports[PortNum].htimrx).Instance->CNT = 0;
  RX_Init_Hvar(PortNum);
  Ports[PortNum].pRxDataPtr[0] = 0;

  /* The timer is configured to capture data with DMA transfer.
   * The number of DMA transfers is set to the maximum possible
  */  
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  __HAL_TIM_ENABLE_DMA(&(Ports[PortNum].htimrx), RX_TIM_DMA_CC(PortNum));
  HAL_TIM_IC_Start(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));

  /* The auxiliary timer is started */
  (Ports[PortNum].htimcountrx).Instance->CNT = 11;
  HAL_TIM_OC_Start_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum));
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
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
  RX_COUNTTIM_IRQHandler(0);
#else
  RX_COUNTTIM_IRQHandler(1);
#endif
}

/**
  * @brief This function handles COUNT_TIM global interrupt.
  * @param  PortNum The current port number
  */
__STATIC_INLINE void RX_COUNTTIM_IRQHandler(uint8_t PortNum)
{
  /* Actions are managed by callbacks */
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum) );
}

/**
  * @brief This function handles ADC interrupts
  */
void ADC1_COMP_IRQHandler(void)
{
  /* The AWD callback is called */
  HAL_ADC_IRQHandler(&usbpdm1_hadc);
}

/* -------------------------------------------------------------------------- */
/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
  * @brief  This function handles TIM interrupts requests.
  * @param  htim : TIM  handle
  * @retval None
  */
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit)
{
  /* Capture compare event */
  if(__HAL_TIM_GET_FLAG(htim, Flag) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, Timit) !=RESET)
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
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
      HAL_TIM_PeriodElapsedCallback(htim);
    }
  }
}

/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
