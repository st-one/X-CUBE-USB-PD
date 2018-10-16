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
#include "string.h"

#include "STUSB1602_Driver.h"
#include "usbpd_porthandle.h"
#include "stm32f0xx_nucleo.h"
#include "p-nucleo-usb002.h"


#if SPI_NSS_LL_IRQPRIORITY(0) <= RX_COUNTTIMIRQ_PRIO(0)
#error "Priority error on Port 0"
#endif
#if SPI_NSS_LL_IRQPRIORITY(1) <= RX_COUNTTIMIRQ_PRIO(1)
#error "Priority error on Port 1"
#endif


/** @addtogroup STM32_USBPD_LIBRARY
 * @{
 */

/** @addtogroup USBPD_DEVICE
 * @{
 */

/** @addtogroup USBPD_DEVICE_HW_IF
 * @{
 */

/** @addtogroup USBPD_DEVICE_HW_IF_IT
 * @{
 */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
  * @var     GPIO_PinState RxNSSStatus[2]
  * @brief   SPI NSS status in reception phase
  * @details It is an array that stores the previous value of the SPI NSS pins to check if a falling or rising event is occurring
  */
static GPIO_PinState RxNSSStatus[2] = {GPIO_PIN_SET,GPIO_PIN_SET};

extern ADC_HandleTypeDef usbpdm1_hadc;                          /* Handle of ADC peripheral */
extern TIM_HandleTypeDef TimHandle;                             /* Handle of eternal timer for TimerServer */
extern STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];    /* Handle for the ports as defined inside @ref USBPD_HW_IF */


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Optimized functions */
static inline void SINGLE_TIM_IRQHandler(TIM_HandleTypeDef *htim, uint32_t Flag, uint32_t Timit);

/**
  * @brief   EXTI line detection callback
  * @param   GPIO_Pin It specifies the pins connected to EXTI line
  * @note    The parameter can be one of the following values:
  * @arg     USER_BUTTON_PIN      ** Defined in stm32f0xx_nucleo.h **
  * @arg     ALERT_GPIO_PIN(0)    ** Defined in usbpd_porthandle.h **
  * @arg     SPI_NSS_PIN(0)       ** Defined in usbpd_porthandle.h **
  * @arg     ALERT_GPIO_PIN(1)    ** Defined in usbpd_porthandle.h **
  * @arg     SPI_NSS_PIN(1)       ** Defined in usbpd_porthandle.h **
  * @retval None
  */
void USBPD_HW_IF_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t PortNum=0;
  GPIO_PinState NSSCurrentState;
  switch (GPIO_Pin)
  {
  case ALERT_GPIO_PIN(0):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(0));
    break;
    
  case SPI_NSS_PIN(0):
    PortNum = 0;
    
    /* Check the NSS current state and compare it with the stored value*/
    NSSCurrentState = HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum));
    
    if( NSSCurrentState == GPIO_PIN_SET || RxNSSStatus[PortNum] == GPIO_PIN_RESET) 
    {
      /* End of reception */
      PHY_HW_IF_RX_Stop(PortNum);
    }
    if( NSSCurrentState == GPIO_PIN_RESET )
    {
      /* Start of reception */
      PHY_HW_IF_RX_Start(PortNum);
    }
    
    /* store the current NSS status */
    RxNSSStatus[PortNum] = NSSCurrentState;
    
    break;    
    
#if defined(MB1303) && (USBPD_PORT_COUNT == 2)
  case ALERT_GPIO_PIN(1):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(1));
    break;
  
  case SPI_NSS_PIN(1):
    PortNum = 1;

    /* Check the NSS current state and compare it with the stored value*/
    NSSCurrentState = HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum));
    
    if( NSSCurrentState == GPIO_PIN_SET || RxNSSStatus[PortNum] == GPIO_PIN_RESET) 
    {
      /* End of reception */
      PHY_HW_IF_RX_Stop(PortNum);
    }
    if( NSSCurrentState == GPIO_PIN_RESET )
    {
      /* Start of reception */
      PHY_HW_IF_RX_Start(PortNum);
    }   
    
    /* store the current NSS status */
    RxNSSStatus[PortNum] = NSSCurrentState;
    
    break;
#endif
    
  default:
    __NOP();
    break;
  }
}


/**
  * @brief   This function handles DMA interrupts on channels 4, 5, 6 and 7
  * @retval  None
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
    __NOP();
  }
  
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);
}



#if (USBPD_PORT_COUNT == 2)
/**
  * @brief   This function handles DMA interrupts on channels 2 and 3
  * @retval  None
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
    __NOP();
  }
  
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);
}
#endif


/**
  * @brief   This function handles interrupts of RX timer associated to port 0
  * @details This timer is used to trigger the data decoding procedure on port 0
  * @retval  None
  */
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void)
{
  uint8_t PortNum = 0;

  /* Actions are managed by callbacks */
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum) );
}


#if (USBPD_PORT_COUNT == 2)
/**
  * @brief   This function handles interrupts of RX timer associated to port 1
  * @note    This timer is used to trigger the data decoding procedure on port 1
  * @retval  None
  */
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void)
{
  uint8_t PortNum = 1;

  /* Actions are managed by callbacks */
  SINGLE_TIM_IRQHandler(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH_ITFLAG(PortNum), RX_COUNTTIMCH_TIMIT(PortNum) );
}
#endif


/* Optimized functions ------------------------------------------------------ */

/**
  * @brief  This function handles TIM interrupts requests
  * @param  htim Pointer to TIM  handle
  * @param  Flag It checks whether the specified TIM interrupt flag is set or not
  * @param  Timit TIM IT
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
