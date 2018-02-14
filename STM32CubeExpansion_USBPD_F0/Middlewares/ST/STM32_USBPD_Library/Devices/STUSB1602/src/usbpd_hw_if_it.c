/**
  ******************************************************************************
  * @file    usbpd_hw_if_it.c
  * @author  System Lab
  * @brief   This file contains HW interface interrupt routines.
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
#include "usbpd_hw_if.h"
#include "usbpd_pe.h"
#include "usbpd_prl.h"
#include "usbpd_cad.h"
#include "string.h"

#include "STUSB1602_Driver.h"
#include "usbpd_porthandle.h"

#include "stm32f0xx_nucleo.h"

#ifdef MB1303
#include "p-nucleo-usb002.h"
#else
#include "STUSB16xx_EVAL.h"
#endif

#if SPI_NSS_LL_IRQPRIORITY(0) <= RX_COUNTTIMIRQ_PRIO(0)
#error "Priority error on Port 0"
#endif
#if SPI_NSS_LL_IRQPRIORITY(1) <= RX_COUNTTIMIRQ_PRIO(1)
#error "Priority error on Port 1"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern ADC_HandleTypeDef        usbpdm1_hadc;
/* Timer counter variable */
extern TIM_HandleTypeDef TimHandle;  /*!< Handle of eternal timer for TimerServer */
extern STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Optimized functions */
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit);



/**
* @brief  EXTI line detection callback.
* @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
*         This parameter can be one of the following values:
*         USER_BUTTON_PIN      (GPIO_PIN_13)   ** Defined in stm32f0xx_nucleo.h **
*         ALERT_GPIO_PIN(0)    (GPIO_PIN_5)    ** Defined in usbpd_porthandle.h **
* @retval None
*/
void USBPD_HW_IF_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t PortNum=0;
  switch (GPIO_Pin)
  {
  case ALERT_GPIO_PIN(0):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(0));
    break;
    
  case SPI_NSS_PIN(0):
    PortNum = 0;
    
    if(HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET )
    {
      PHY_HW_IF_RX_Start(PortNum);
    }
    else
    {  
      /* End of reception */
      PHY_HW_IF_RX_Stop(PortNum);
    }
    break;    
    
#if defined(MB1303) && (USBPD_PORT_COUNT == 2)
  case ALERT_GPIO_PIN(1):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(1));
    break;
  
  case SPI_NSS_PIN(1):
    PortNum = 1;
    
    if(HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET )
    {
      PHY_HW_IF_RX_Start(PortNum);
    }
    else
    {  
      /* End of reception */
      PHY_HW_IF_RX_Stop(PortNum);
    }
    break;
#endif
    
  default:
    __NOP();
    break;
  }
}


/**
* @brief This function handles DMA1 channel 4, 5, 6 and 7 interrupts.
*/
void USBPD_DMA_PORT0_IRQHandler(void)
{
  uint8_t *end;
  
  /* Handler DMA TX PORT 0  */
  if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[0].hdmatx)) != RESET)
  {
    /* Transfer complete interrupt is used to end the transmission */
    if (Ports[0].State!=HAL_USBPD_PORT_STATE_BIST)
    {
      PHY_HW_IF_TX_Done(0);
    }    
    else
    { 
      Ports[0].BIST_index+=1;
      if(Ports[0].BIST_index == BIST_MAX_LENGTH) 
      {
        /* Get the address of the transmission buffer*/
        end = Ports[0].pTxBuffPtr;
        end[(TX_BUFFER_LEN)*2 - 1] = 0;
        STUSB16xx_HW_IF_Set_DMA_Normal_Mode(0); 
      }
      if(Ports[0].BIST_index > BIST_MAX_LENGTH) 
      {
        PHY_HW_IF_TX_Done(0);
        Ports[0].BIST_index=0;
      }
    }
  }
  /* Handler DMA RX PORT 0  */
  else if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[0].hdmarx)) != RESET)
  {
//    USBPD_BSP_LED_Toggle(ELED2);  
  }
  
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);
}



#if (USBPD_PORT_COUNT == 2)
/**
* @brief This function handles DMA1 channel 2 and 3 interrupts.
*/
void USBPD_DMA_PORT1_IRQHandler(void)
{
  uint8_t *end;
  
  /* Handler DMA TX PORT 1  */
  if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[1].hdmatx)) != RESET)
  {
    /* Transfer complete interrupt is used to end the transmission */
    if (Ports[1].State!=HAL_USBPD_PORT_STATE_BIST)
    {
      PHY_HW_IF_TX_Done(1);
    }    
    else
    { 
      Ports[1].BIST_index+=1;
      if(Ports[1].BIST_index == BIST_MAX_LENGTH) 
      {
        /* Get the address of the transmission buffer*/
        end = Ports[1].pTxBuffPtr;
        end[(TX_BUFFER_LEN)*2 - 1] = 0;
        STUSB16xx_HW_IF_Set_DMA_Normal_Mode(1); 
      }
      if(Ports[1].BIST_index > BIST_MAX_LENGTH) 
      {
        PHY_HW_IF_TX_Done(1);
        Ports[1].BIST_index=0;
      }
    }
  }
  /* Handler DMA RX PORT 0  */
  else if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[1].hdmarx)) != RESET)
  {
//    USBPD_BSP_LED_Toggle(ELED2);  
  }
  
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);
}
#endif


/**
* @brief This function handles COUNT_TIM global interrupt.
*/
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void)
{
  uint8_t PortNum = 0;

  /* Actions are managed by callbacks */
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum) );
}


#if (USBPD_PORT_COUNT == 2)
/**
* @brief This function handles COUNT_TIM global interrupt.
*/
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void)
{
  uint8_t PortNum = 1;

  /* Actions are managed by callbacks */
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum) );
}
#endif

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
    if(__HAL_TIM_GET_IT_SOURCE(htim, Timit) != RESET)
    {
        __HAL_TIM_CLEAR_IT(htim, Timit);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
        /* Output compare event */
        HAL_TIM_OC_DelayElapsedCallback(htim);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* TIM Update event */
//  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
//  {
//    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET)
//    {
//      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
//      HAL_TIM_PeriodElapsedCallback(htim);
//    }
//  }
}

/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
