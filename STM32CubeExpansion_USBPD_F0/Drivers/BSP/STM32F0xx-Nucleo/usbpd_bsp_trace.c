/**
  ******************************************************************************
  * @file    usbpd_bsp_trace.c
  * @author  MCD Application Team
  * @brief   This file contains phy interface control functions.
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
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"
#include "usbpd_bsp_trace.h"

/* Private typedef -----------------------------------------------------------*/
/* Private Defines */
#define USBPD_BSP_TRACE_C
#define TRACE_BAUDRATE    921600u /*460800u*/  /*115200*/

#if USBPD_PORT_COUNT==1||defined(USBPD_TCPM_MODULE_ENABLED)
#define TRACE_TX_DMA
#endif /* USBPD_PORT_COUNT=1 || USBPD_TCPM_MODULE_ENABLED */

#if defined(USBPD_TCPM_MODULE_ENABLED)
/* Configuration used with OnSemi board (EVAL_FUSB307)*/
#define TRACE_USART_INSTANCE                    USART4
#define TRACE_CLK_USART                         LL_APB1_GRP1_PERIPH_USART4
#define TRACE_ENABLE_CLK_USART(_INSTANCE_)      LL_APB1_GRP1_EnableClock(_INSTANCE_)
#define TRACE_USART_IRQ                         USART3_4_IRQn
#define TRACE_TX_GPIO                           GPIOA
#define TRACE_TX_PIN                            LL_GPIO_PIN_0
#define TRACE_TX_AF                             LL_GPIO_AF_4
#define TRACE_TX_AF_FUNCTION                    LL_GPIO_SetAFPin_0_7
#define TRACE_RX_PIN                            LL_GPIO_PIN_1
#define TRACE_RX_AF                             LL_GPIO_AF_4
#define TRACE_RX_AF_FUNCTION                    LL_GPIO_SetAFPin_0_7
#define TRACE_TX_DMA_CHANNEL                    LL_DMA_CHANNEL_7
#define TRACE_TX_DMA_IRQ                        DMA1_Channel4_5_6_7_IRQn
#define TRACE_TX_DMA_ACTIVE_FLAG                LL_DMA_IsActiveFlag_TC7
#define TRACE_TX_DMA_CLEAR_FLAG                 LL_DMA_ClearFlag_GI7
#else
#define TRACE_USART_INSTANCE                    USART1
#define TRACE_CLK_USART                         LL_APB1_GRP2_PERIPH_USART1
#define TRACE_ENABLE_CLK_USART(_INSTANCE_)      LL_APB1_GRP2_EnableClock(_INSTANCE_)
#define TRACE_USART_IRQ                         USART1_IRQn
#define TRACE_TX_GPIO                           GPIOA
#define TRACE_TX_PIN                            LL_GPIO_PIN_9
#define TRACE_TX_AF                             LL_GPIO_AF_1
#define TRACE_TX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACE_RX_PIN                            LL_GPIO_PIN_10
#define TRACE_RX_AF                             LL_GPIO_AF_1
#define TRACE_RX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACE_TX_DMA_CHANNEL                    LL_DMA_CHANNEL_2
#define TRACE_TX_DMA_IRQ                        DMA1_Channel2_3_IRQn
#define TRACE_TX_DMA_ACTIVE_FLAG                LL_DMA_IsActiveFlag_TC2
#define TRACE_TX_DMA_CLEAR_FLAG                 LL_DMA_ClearFlag_GI2
#endif /* USBPD_TCPM_MODULE_ENABLED */

/* Private Variable */
void            (*fptr_tx)(void) = NULL;
void            (*fptr_rx)(uint8_t, uint8_t) = NULL;
#if !defined(TRACE_TX_DMA)
static uint8_t *PtrDataU = NULL;
static uint32_t GSize = 0;
#endif /*TRACE_TX_DMA*/

/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  Trace init
  * @param  callbackTX
  * @param  callbackRX
  * @retval none
  */
void BSP_TRACE_Init(void (*callbackTX)(void), void (*callbackRX)(uint8_t, uint8_t))
{
  fptr_tx = callbackTX;
  fptr_rx = callbackRX;
  
  /* Enable the peripheral clock of GPIO Port */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(TRACE_TX_GPIO, TRACE_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  TRACE_TX_AF_FUNCTION(TRACE_TX_GPIO, TRACE_TX_PIN, TRACE_TX_AF);
  LL_GPIO_SetPinSpeed(TRACE_TX_GPIO, TRACE_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(TRACE_TX_GPIO, TRACE_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(TRACE_TX_GPIO, TRACE_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(TRACE_TX_GPIO, TRACE_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  TRACE_RX_AF_FUNCTION(TRACE_TX_GPIO, TRACE_RX_PIN, TRACE_RX_AF);
  LL_GPIO_SetPinSpeed(TRACE_TX_GPIO, TRACE_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(TRACE_TX_GPIO, TRACE_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(TRACE_TX_GPIO, TRACE_RX_PIN, LL_GPIO_PULL_UP);

  /* Enable the peripheral clock for USART */
  TRACE_ENABLE_CLK_USART(TRACE_CLK_USART);

  /* Configure USART */
  
  /* Disable USART prior modifying configuration registers */
  LL_USART_Disable(TRACE_USART_INSTANCE);
  /* TX/RX direction */
  LL_USART_SetTransferDirection(TRACE_USART_INSTANCE, LL_USART_DIRECTION_TX_RX);
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(TRACE_USART_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  /* Oversampling by 16 */
  LL_USART_SetOverSampling(TRACE_USART_INSTANCE, LL_USART_OVERSAMPLING_16);
  /* Set Baudrate to 115200 using APB frequency set to 80000000 Hz */
  /* This frequency can also be calculated through LL RCC macro */
  /* Ex :
      pllclk = __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6), LL_RCC_PLLM_DIV1, 40, LL_RCC_PLLR_DIV2);
      hclk = __LL_RCC_CALC_HCLK_FREQ(pllclk, LL_RCC_GetAHBPrescaler());
      periphclk = __LL_RCC_CALC_PCLKx_FREQ(hclk, LL_RCC_GetAPBxPrescaler());  x=1 or 2 depending on USART instance

      periphclk is expected to be equal to 80000000 Hz

      In this example, Peripheral Clock is equal to SystemCoreClock
  */

  uint32_t _pllclk = __LL_RCC_CALC_PLLCLK_FREQ(HSI48_VALUE, LL_RCC_PLL_MUL_2, LL_RCC_PREDIV_DIV_2);
  uint32_t _hclk = __LL_RCC_CALC_HCLK_FREQ(_pllclk, LL_RCC_GetAHBPrescaler());
  uint32_t _periphclk = __LL_RCC_CALC_PCLK1_FREQ(_hclk, LL_RCC_GetAPB1Prescaler());

  LL_USART_SetBaudRate(TRACE_USART_INSTANCE, _periphclk, LL_USART_OVERSAMPLING_16, TRACE_BAUDRATE);
  LL_USART_Enable(TRACE_USART_INSTANCE);

  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(TRACE_USART_INSTANCE))) || (!(LL_USART_IsActiveFlag_REACK(TRACE_USART_INSTANCE))))
  {
  }

#if defined(TRACE_TX_DMA)
  /* Configure TX DMA */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  
  /* (3) Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigTransfer(DMA1, TRACE_TX_DMA_CHANNEL,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_HIGH               |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);
  
  LL_DMA_EnableIT_TC(DMA1, TRACE_TX_DMA_CHANNEL);
  LL_DMA_EnableIT_TE(DMA1, TRACE_TX_DMA_CHANNEL);

  /* Configure the interrupt for TX */
  NVIC_SetPriority(TRACE_TX_DMA_IRQ, 3);
  NVIC_EnableIRQ(TRACE_TX_DMA_IRQ);
#endif /*TRACE_TX_DMA*/
  
  LL_USART_EnableIT_PE(TRACE_USART_INSTANCE);
  LL_USART_EnableIT_RTO(TRACE_USART_INSTANCE);
  LL_USART_EnableIT_LBD(TRACE_USART_INSTANCE);
#if defined(TRACE_TX_DMA)
  LL_USART_DisableIT_TC(TRACE_USART_INSTANCE);
#endif /*TRACE_TX_DMA*/
  LL_USART_DisableIT_CTS(TRACE_USART_INSTANCE);
  LL_USART_DisableIT_WKUP(TRACE_USART_INSTANCE);
  LL_USART_DisableIT_LBD(TRACE_USART_INSTANCE);
  LL_USART_RequestTxDataFlush(TRACE_USART_INSTANCE);

#if !defined(_GUI_INTERFACE)
  /* Enable USART IT for RX */
  LL_USART_EnableIT_RXNE(TRACE_USART_INSTANCE);
  LL_USART_EnableIT_ERROR(TRACE_USART_INSTANCE);

  // Configure the interrupt 
  NVIC_SetPriority(TRACE_USART_IRQ, 3);
  NVIC_EnableIRQ(TRACE_USART_IRQ);
#endif /* !_GUI_INTERFACE */

  return;
}

/**
  * @brief  Trace Deinit
  * @retval none
  */
void BSP_TRACE_DeInit(void)
{
  return;
}

/**
  * @brief  Allow to update the RX callback
  * @param  callbackRX
  * @retval none
  */
void BSP_TRACE_RegisterRxCallback(void (*callbackRX)(uint8_t, uint8_t))
{
  fptr_rx = callbackRX;
}

/**
  * @brief  Start RX reception only when OSKernel have been started
  * @retval none
  */
void BSP_TRACE_StartRX(void)
{
  /* Enable USART IT for RX */
  LL_USART_EnableIT_RXNE(TRACE_USART_INSTANCE);
  LL_USART_EnableIT_ERROR(TRACE_USART_INSTANCE);

  /* Configure the interrupt for RX */
  HAL_NVIC_SetPriority(TRACE_USART_IRQ, 3 ,0U);
  NVIC_EnableIRQ(TRACE_USART_IRQ);
}

/**
  * @brief  Function to handle reception in DMA mode
  * @retval none
  */
void BSP_TRACE_IRQHandlerDMA(void)
{
  if(TRACE_TX_DMA_ACTIVE_FLAG(DMA1))
  {
    TRACE_TX_DMA_CLEAR_FLAG(DMA1);
    LL_DMA_DisableChannel(DMA1, TRACE_TX_DMA_CHANNEL);
    /* call the callback */
    if(NULL != fptr_tx) fptr_tx();
  }
}

void BSP_TRACE_IRQHandlerUSART(void)
{
#if !defined(TRACE_TX_DMA)
  /* Ready to send  */
  if(LL_USART_IsActiveFlag_TXE(TRACE_USART_INSTANCE) && LL_USART_IsEnabledIT_TXE(TRACE_USART_INSTANCE))
  {
    (TRACE_USART_INSTANCE)->TDR = *PtrDataU;
    
    if(GSize > 0)
    {
      PtrDataU++;
      GSize--;
    }
    if (GSize == 0)
    {
      LL_USART_DisableIT_TXE(TRACE_USART_INSTANCE);
      
      /* call the callback*/
      if(NULL != fptr_tx) fptr_tx();
    }
  }
#endif /*TRACE_TX_DMA*/
  /* Ready to read reception*/
  if(LL_USART_IsActiveFlag_RXNE(TRACE_USART_INSTANCE) && LL_USART_IsEnabledIT_RXNE(TRACE_USART_INSTANCE))
  {
    __IO uint32_t received_char;

    /* Read Received character. RXNE flag is cleared by reading of RDR register */
    received_char = LL_USART_ReceiveData8(TRACE_USART_INSTANCE);
    if(fptr_rx != NULL) 
    {
      fptr_rx(received_char, 0);
    }
  }
  
  if( (LL_USART_IsActiveFlag_PE(TRACE_USART_INSTANCE) && LL_USART_IsEnabledIT_PE(TRACE_USART_INSTANCE))            /* Parity error    */
     || (LL_USART_IsActiveFlag_RTO(TRACE_USART_INSTANCE) && LL_USART_IsEnabledIT_RTO(TRACE_USART_INSTANCE))        /* Receiver timeout*/
       || (LL_USART_IsEnabledIT_ERROR(TRACE_USART_INSTANCE) && LL_USART_IsActiveFlag_FE(TRACE_USART_INSTANCE))     /* Framing error   */
         || (LL_USART_IsEnabledIT_ERROR(TRACE_USART_INSTANCE) && LL_USART_IsActiveFlag_ORE(TRACE_USART_INSTANCE))  /* Overrun error   */
           || (LL_USART_IsEnabledIT_ERROR(TRACE_USART_INSTANCE) && LL_USART_IsActiveFlag_NE(TRACE_USART_INSTANCE)))/* Noise detection */
  {
	/* Flags clearing */
    LL_USART_ClearFlag_PE(TRACE_USART_INSTANCE);
    LL_USART_ClearFlag_RTO(TRACE_USART_INSTANCE);
    LL_USART_ClearFlag_FE(TRACE_USART_INSTANCE);
    LL_USART_ClearFlag_ORE(TRACE_USART_INSTANCE);
    LL_USART_ClearFlag_NE(TRACE_USART_INSTANCE);
    
    if(fptr_rx != NULL) { fptr_rx(1, 1);  /* 1 indicate a reception error */ }
  }
}

void BSP_TRACE_SendData(uint8_t *pData, uint32_t Size)
{
#if !defined(TRACE_TX_DMA)
  PtrDataU = pData;
  GSize = Size;

  LL_USART_EnableIT_TXE(TRACE_USART_INSTANCE);
#else
  LL_DMA_ConfigAddresses(DMA1, TRACE_TX_DMA_CHANNEL,
                         (uint32_t)pData,
                         LL_USART_DMA_GetRegAddr(TRACE_USART_INSTANCE, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, TRACE_TX_DMA_CHANNEL));
  LL_DMA_SetDataLength(DMA1, TRACE_TX_DMA_CHANNEL, Size);
  
  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(TRACE_USART_INSTANCE);
  
  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, TRACE_TX_DMA_CHANNEL);
  
  LL_USART_EnableDirectionTx(TRACE_USART_INSTANCE);
#endif /*TRACE_TX_DMA*/
  return;
}

#if 0
USBPD_StatusTypeDef BSP_TRACE_SaveDataInFlash(void)
{
  USBPD_StatusTypeDef status = USBPD_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t page_error;
  
  /* Disable interrupts */
  __disable_interrupt();
  
  /* Init Flash registers for writing */
  HAL_FLASH_Unlock();
  
  /* Erase the page associated to the GUI parameters */
  erase_init.TypeErase          = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress        = ADDR_FLASH_LAST_PAGE;
  erase_init.NbPages            = 1;
  
  status = (USBPD_StatusTypeDef)HAL_FLASHEx_Erase(&erase_init, &page_error);
  
  /* If Erase is OK, program the new data */
  if ((0xFFFFFFFF == page_error) && (USBPD_OK == status))
  {
    /* Save the nb of sink and src PDO */
    uint64_t value;
    value = PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.NumberOfPDO
                   | (PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.NumberOfPDO << 8);
#if USBPD_PORT_COUNT==2
   value |= (PWR_Port_PDO_Storage[USBPD_PORT_1].SinkPDO.NumberOfPDO   << 16)
                   | (PWR_Port_PDO_Storage[USBPD_PORT_1].SourcePDO.NumberOfPDO << 24);
#endif /* USBPD_PORT_COUNT==2 */
    status = (USBPD_StatusTypeDef)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, GUI_FLASH_ADDR_NB_PDO_SNK_P0, value);
    
    /* Save PORT0_PDO_ListSRC */
    if (USBPD_OK == status)
    {
      status = TRACE_SavePDOInFlash(GUI_FLASH_ADDR_PDO_SRC_P0, PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.ListOfPDO);
    }

    /* Save PORT0_PDO_ListSNK */
    if (USBPD_OK == status)
    {
      status = TRACE_SavePDOInFlash(GUI_FLASH_ADDR_PDO_SNK_P0, PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.ListOfPDO);
    }

#if USBPD_PORT_COUNT==2
    /* Save PORT1_PDO_ListSRC */
    if (USBPD_OK == status)
    {
      status = TRACE_SavePDOInFlash(GUI_FLASH_ADDR_PDO_SRC_P1, PWR_Port_PDO_Storage[USBPD_PORT_1].SourcePDO.ListOfPDO);
    }

    /* Save PORT1_PDO_ListSNK */
    if (USBPD_OK == status)
    {
      status = TRACE_SavePDOInFlash(GUI_FLASH_ADDR_PDO_SNK_P1, PWR_Port_PDO_Storage[USBPD_PORT_1].SinkPDO.ListOfPDO);
    }
#endif /* USBPD_PORT_COUNT==2 */

    /* Save DPM_Settings of port 0 */
    if (USBPD_OK == status)
    {
      status = TRACE_SaveSettingsInFlash(GUI_FLASH_ADDR_DPM_SETTINGS, (uint32_t*)DPM_Settings, sizeof(USBPD_SettingsTypeDef));
    }

    /* Save DPM_Settings of port 0 */
    if (USBPD_OK == status)
    {
      status = TRACE_SaveSettingsInFlash(GUI_FLASH_ADDR_DPM_USER_SETTINGS, (uint32_t*)DPM_USER_Settings, sizeof(USBPD_USER_SettingsTypeDef));
    }

#if defined(_VDM)
    /* Save DPM_Settings of port 0 */
    if (USBPD_OK == status)
    {
      status = TRACE_SaveSettingsInFlash(GUI_FLASH_ADDR_DPM_VDM_SETTINGS, (uint32_t*)DPM_VDM_Settings, sizeof(USBPD_VDM_SettingsTypeDef));
    }
#endif /* _VDM */
  }

  /* Lock the flash afer end of operations */
  HAL_FLASH_Lock();
  
  /* Disable interrupts */
  __enable_interrupt();
  
  return status;
}

static USBPD_StatusTypeDef TRACE_SavePDOInFlash(uint32_t Address, uint32_t *pListOfPDO)
{
  uint64_t data_in_64;
  uint32_t index, index_flash;
  uint32_t value[2];
  USBPD_StatusTypeDef status = USBPD_OK;
  
  /* Save PORT0_PDO_ListSRC */
  for (index = 0, index_flash = 0; ((index < USBPD_MAX_NB_PDO) && (USBPD_OK == status)); index++, index_flash++)
  {
    value[0] = pListOfPDO[index];
    index++;
    if (index < USBPD_MAX_NB_PDO)
    {
      value[1] = pListOfPDO[index];
    }
    else
    {
      value[1] = (0xFFFFFFFF);
    }
    
    data_in_64 = value[0] | (uint64_t)value[1] << 32;
    
    /* Save in the FLASH */
    status = (USBPD_StatusTypeDef)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (Address + (8 * index_flash)) , data_in_64);
  }
  return status;
}

static USBPD_StatusTypeDef TRACE_SaveSettingsInFlash(uint32_t Address, uint32_t *pSettings, uint32_t Size)
{
  uint64_t data_in_64;
  uint32_t index, index_flash;
  uint32_t value[2];
  USBPD_StatusTypeDef status = USBPD_OK;
  
  uint32_t nb_double = ((Size * USBPD_PORT_COUNT) / 4);
  uint8_t remaining = ((Size * USBPD_PORT_COUNT) % 4);

  /* Save Settings in the FLASH */
  for (index = 0, index_flash = 0; ((index < nb_double) && (USBPD_OK == status)); index++, index_flash++)
  {
    value[0] = pSettings[index];
    index++;
    if (index < nb_double)
    {
      value[1] = pSettings[index];
    }
    else
    {
      if (0 == remaining)
        value[1] = (0xFFFFFFFF);
      else
        while(1);
    }
    
    data_in_64 = value[0] | (uint64_t)value[1] << 32;
    
    /* Save in the FLASH */
    status = (USBPD_StatusTypeDef)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (Address + (8 * index_flash)) , data_in_64);
  }
  return status;
}
#endif /* _GUI_INTERFACE */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
