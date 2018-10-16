/**
  ******************************************************************************
  * @file    usbpd_phy_hw_if.c
  * @author  System Lab
  * @brief   This file contains power interface control functions.
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
#include "usbpd_timersserver.h"
#include "usbpd_porthandle.h"
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"
#include "p-nucleo-usb002.h"


/** @addtogroup STM32_USBPD_LIBRARY
 * @{
 */

/** @addtogroup USBPD_DEVICE
 * @{
 */

/** @addtogroup USBPD_DEVICE_HW_IF
 * @{
 */

/** @addtogroup USBPD_DEVICE_PHY_HW_IF
 * @{
 */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define   TX_PREAMBLE             0xAA    /*!< Preamble byte 0xAA = 10101010b */
#define   TX_PREAMBLE_SIZE        8       /*!< Amount of repeated preamble bytes */
#define   TX_BUFFER_SIZE          56      /*!< Amount of bytes constituting the packet to be transmitted */
#define   RX_BUFFER_SIZE          60      /*!< Amount of bytes constituting the packet to be received */

/* Private variables ---------------------------------------------------------*/
uint8_t         RXBuffer0[RX_BUFFER_SIZE];      /*!< Buffer storing raw received data on port 0 */
uint8_t         TXBuffer0[TX_BUFFER_SIZE];      /*!< Buffer that stores the packet to be transmitted on port 0 */
#if (USBPD_PORT_COUNT == 2)
uint8_t         RXBuffer1[RX_BUFFER_SIZE];      /*!< Buffer storing raw received data on port 1 */
uint8_t         TXBuffer1[TX_BUFFER_SIZE];      /*!< Buffer that stores the packet to be transmitted on port 1 */
#endif

CRC_HandleTypeDef   hcrc;                   /*!< Handle of CRC peripheral */
ADC_HandleTypeDef   usbpdm1_hadc;           /*!< Handle of ADC peripheral */
DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */

uint8_t preamble_offset;                        /*!< Offset identifies the byte of data after preamble */
uint8_t preamble_index;                         /*!< Index identifies the bit of data after preamble */
uint32_t preamble_counter = 0;                  /*!< Preamble counter */

uint32_t   ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]; /*!< Array that stores ADC sampled values */

/* Array of ADC CH pins used by @ref STUSB16xx_EVAL */
static const USBPD_BSP_GPIOPins_TypeDef USBPD_ADCs[USBPD_ADCn] =
{
  USBPD_BSP_ADC(GPIOC, 4, ADC_CHANNEL_14), /*ADC0 on schematic - VBUS port0*/
  USBPD_BSP_ADC(GPIOA, 7, ADC_CHANNEL_7),  /*ADC1 on schematic - IBUS port0*/
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOB, 0, ADC_CHANNEL_8),  /*ADC2 on schematic - VBUS port1*/
  USBPD_BSP_ADC(GPIOA, 4, ADC_CHANNEL_4),  /*ADC3 on schematic - IBUS port1*/
#endif
  USBPD_BSP_ADC(GPIOC, 0, ADC_CHANNEL_10)  /*ADC4 on schematic - PowConn17*/
  
};

extern STUSB16xx_PORT_HandleTypeDef Ports[]; /* Handle for the ports inside @ref USBPD_DEVICE_HW_IF */
uint8_t ud_index_current[3] = {0, 0,0};
uint8_t modulo=0;


/* Private function prototypes -----------------------------------------------*/

/* Unwrap data init function */
void HW_IF_UnwrapData_Init(uint8_t PortNum);

/* Common peripheral Init function*/
void HW_IF_CRC_Init(void);
void HW_IF_ADC_Init(void);


/* Inner functions prototypes ------------------------------------------------*/
static inline void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum);


/* Public functions ----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Public_Functions USBPD DEVICE PHY HW IF Public functions
  * @details Public functions can be used at stack level
  * @{
  */


/**
  * @brief  Global hardware initialization
  * @retval None
  */
void USBPD_HW_IF_GlobalHwInit(void)
{
  /* Configure the ADCx peripheral */
  HW_IF_ADC_Init();

  /* Run the ADC calibration */
  HAL_ADCEx_Calibration_Start(&usbpdm1_hadc);

  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&usbpdm1_hadc, ADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE);

  HW_IF_CRC_Init();
  HW_IF_PWR_DigitalGPIO_Init();

  /* used for the PRL/PE timing */
  USBPD_TIM_Init();
}


/**
  * @brief  CRC calculation
  * @param  *pBuffer    Pointer to the input data buffer
  * @param  len         Input data buffer length
  * @retval uint32_t    CRC value
  */
uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint8_t len)
{
  uint32_t crc;

  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)pBuffer, len);
  crc ^= 0xFFFFFFFF;
  return crc;
}


/**
* @brief  It attaches the preamble at the beginning of the packet and moves it towards the SPI  
* @param  PortNum       The port index
* @param  *pBuffer      Pointer to the TX data buffer
* @param  Bitsize:      Amount of bits to be transmitted
* @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize)
{
  /* Check if the port is still receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
    return USBPD_BUSY;

  uint8_t *pTxBuffer = (uint8_t *)Ports[PortNum].pTxBuffPtr;
  uint16_t size = DIV_ROUND_UP(Bitsize, 8)+TX_PREAMBLE_SIZE;

  memset((uint8_t *)pTxBuffer, 0x00, TX_BUFFER_SIZE);
  memset((uint8_t *)pTxBuffer, TX_PREAMBLE, TX_PREAMBLE_SIZE);                          /* preamble is added */
  memcpy((uint8_t *)(pTxBuffer+TX_PREAMBLE_SIZE), pBuffer, (size-TX_PREAMBLE_SIZE));    /* data are added */

  /* Spare clock cycles at the end of transmission are calculated */
  Ports[PortNum].TxSpareBits = (Bitsize % 8);

  /* Packet is ready to be sent to SPI */
  USBPD_StatusTypeDef ret = USBPD_OK;
  ret = STUSB16xx_HW_IF_Send_Packet(PortNum, pTxBuffer, size);
  return ret;
}


/**
* @brief  It sends BIST pattern  
* @param  PortNum       The port index
* @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* BIST Carrier mode flag set */
  Ports[PortNum].State=HAL_USBPD_PORT_STATE_BIST;
  
  /* Fill the buffer with the pattern to be sent */
  memset(Ports[PortNum].pTxBuffPtr, 0xAA, TX_BUFFER_LEN*2);
  
  /* start a circular DMA transfer */
  STUSB16xx_HW_IF_Set_DMA_Circular_Mode(PortNum);
  
  /* Set the SPI in TX mode */
  STUSB16xx_HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);
  
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);
  
  /* Send TX Buffer by SPI DMA */
  HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, (uint8_t*)(Ports[PortNum].pTxBuffPtr), TX_BUFFER_LEN*2);
  
  /* Start transmission */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
  
  ret = USBPD_OK;
  return ret;
}


/**
  * @brief  HAL_TIM_OC_DelayElapsedCallback
  * @param  *htim
  * @retval void
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  
  uint8_t PortNum, r, prev_bit, curr_bit;
  uint32_t temp_data = 0;
  
  /* Get the port number from the htim peripheral */
#if (USBPD_PORT_COUNT==1)
  PortNum = 0;
#elif (USBPD_PORT_COUNT==2)
  PortNum = ( (htim->Instance == RX_COUNTTIM(0)) ? 0 : 1);
#endif /* RX TIMER IDENTIFIED */
  
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pRxBuffPtr;
  uint32_t tim_count = Ports[PortNum].htimcountrx.Instance->CNT;
  
  while (
         (ud->exed_flag == 0) &&
           ((RX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR)>(ud->index + 2)) &&                /* checking if in the buffer the are enough data */
             (Ports[PortNum].htimcountrx.Instance->CNT < tim_count + DMA_TIME_TASK) &&
               (HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET)        /* stopping the decoding in case of NSS is high */
                 )
  {
    
    if (pbuff_in[ud->index]==0xFF)
    {
      __NOP();
    }
    
    if (!ud->preamble)                  /* The end of preamble hasn't identified yet */ 
    {
      /* Search end of preamble */
      r = pbuff_in[ud->index]^0xAA;
      if (r == 0x00 || r == 0xFF)       /* The end of preamble is not part of the received data */ 
      {
        /* Preamble */
        ud->index++;
      }
      else                              /* Received data contain the end of preamble */
      {
        prev_bit = (pbuff_in[ud->index-1]>>7) & 0x01;
        while (ud->offset < 8)
        {
          curr_bit = (pbuff_in[ud->index]>>ud->offset) & 0x01;
          if (prev_bit == curr_bit)
          {
            /* Preamble identified. Index and offset identify the byte and the bit position of data after preamble */
            if (curr_bit == 0)
            {
              if (ud->offset == 0)
              {
                ud->offset = 7;
                ud->index--;
              }
              else
                ud->offset--;
            }
            ud->preamble = 1;
            preamble_offset = ud->offset;
            preamble_index = ud->index;
            break;
          }
          prev_bit = curr_bit;
          ud->offset++;
        }
      }
    }
    else
    {
      /* Extract 10-bits from data flow (structured in bytes) */
      __NOP();
      memcpy(&temp_data, &pbuff_in[ud->index], 3);
      temp_data = (temp_data >> ud->offset) & 0x3FF;
      ud->index += ud->offset <= 5 ? 1 : 2;
      ud->offset = (ud->offset + 2) & 7;
      
      /* Callback core phy accumulate */
      if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate != NULL)
      {
        Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, temp_data);
      }
      
      /* EOP detecting */
      if ((temp_data & 0x1F) == 0x0D) /* EOP */
      {
        /* EOP to be managed */
        ud->exed_flag = 2;
      }
    }
  }
  
  /* NB, MB */
  Ports[PortNum].ud_index_current[Ports[PortNum].modulo] = ud->index ;
  Ports[PortNum].modulo++;
  Ports[PortNum].modulo = Ports[PortNum].modulo%3;
  
  if (
      (Ports[PortNum].ud_index_current[Ports[PortNum].modulo] == Ports[PortNum].ud_index_current[(Ports[PortNum].modulo+1)%3]) &&
        (Ports[PortNum].ud_index_current[(Ports[PortNum].modulo+1)%3] == Ports[PortNum].ud_index_current[(Ports[PortNum].modulo+2)%3]) &&
      (ud->index > 5)
   )
  {
    ud->exed_flag = 3;
  }
  
  if (ud->exed_flag == 2)
  {
    PHY_HW_IF_RX_Stop(PortNum);
  }
}


/**
 * @brief   It performs actions at harware level when a RESET event occurs
 * @details Not implemented
 * @param   PortNum The port index
 * @param   Mode Allowed values are ACKNOWLEDGE or REQUEST
 * @retval  None
 */
void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode)
{
  __NOP();
}


/**
 * @brief   It acts on the state of GPIO pin
 * @param   gpio The GPIO structure
 * @param   PinState Allowed values are GPIO_PIN_RESET or GPIO_PIN_SET
 * @retval  None
 */
void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(gpio.GPIOx, gpio.GPIO_Pin, PinState);
}


/**
 * @brief   It sets the state of GPIO pin
 * @param   gpio The GPIO structure
 * @retval  None
 */
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Sets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_SET);
}


/**
 * @brief   It resets the state of GPIO pin
 * @param   gpio The GPIO structure
 * @retval  None
 */
void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Resets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_RESET);
}


/**
 * @brief   It toggles the state of GPIO pin
 * @param   gpio The GPIO structure
 * @retval  None
 */
void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Toggle the pin */
  HAL_GPIO_TogglePin(gpio.GPIOx, gpio.GPIO_Pin);
}


/** @}*/ // End of PUBLIC_FUNCTIONS group


/* Private functions ---------------------------------------------------------*/


/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Private_Functions USBPD DEVICE PHY HW IF Private functions
  * @details Private functions can be used at hardware interface level
  * @{
  */


/**
 * @brief  Initialization of ADC analog GPIOs
 * @retval None
 */
void PHY_HW_IF_ADCAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;

  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    GPIO_InitStruct.Pin = USBPD_ADCs[ch].GPIO_Pin;
    HAL_GPIO_Init(USBPD_ADCs[ch].GPIOx, &GPIO_InitStruct);
  }
}


/**
 * @brief  Deinitialization of ADC analog GPIOs.
 * @retval None
 */
void PHY_HW_IF_ADCAnalogGPIO_DeInit(void)
{
  uint8_t ch = 0;
  /* De-initialize GPIO pin of the selected ADC channel */

  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    HAL_GPIO_DeInit(USBPD_ADCs[ch].GPIOx, USBPD_ADCs[ch].GPIO_Pin);
  }
}


/**
  * @brief  Initialization of the ADC DMA
  * @retval None
  */
void PHY_HW_IF_ADCDMA_Init(void)
{
  /* Configuration of DMA parameters */
  DmaHandle.Instance = ADCx_DMA;

  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;       /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;

  /* Initialization of the DMA associated to the peripheral */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Association of the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(&usbpdm1_hadc, DMA_Handle, DmaHandle);
}


/**
  * @brief  Deinitialization of the ADC DMA
  * @retval None
  */
void PHY_HW_IF_ADCDMA_DeInit(void)
{
  /* Deinitialization of the DMA associated to the peripheral */
  if(usbpdm1_hadc.DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(usbpdm1_hadc.DMA_Handle);
  }
}



/**
  * @brief  PHY is prepared for data receiving phase
  * @param  PortNum The port index
  * @retval None
  */
void PHY_HW_IF_RX_Start(uint8_t PortNum)
{
  /* Callback core phy reset */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset(PortNum);
  }

  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_RX;

  /* Variables for decoding stage are initalized */
  HW_IF_UnwrapData_Init(PortNum);

    HAL_SPI_DMAStop(&Ports[PortNum].hspi);

  /* Clean-up of SPI DR before starting the receiving phase */
  SPI_HandleTypeDef * local_hspi = &Ports[PortNum].hspi;
  __IO uint32_t a = 0x0;
  while(local_hspi->Instance->SR & 0x01)
  {
    a = local_hspi->Instance->DR;
    UNUSED(a);
  }

  /* Clean-up of buffer storing raw received data */
  memset(Ports[PortNum].pRxBuffPtr, 0x00, RX_BUFFER_SIZE);
  
  /* Start DMA receiving */
  HAL_SPI_Receive_DMA(&Ports[PortNum].hspi, (uint8_t*)Ports[PortNum].pRxBuffPtr, RX_BUFFER_SIZE);

  /* reset blabla*/
  Ports[PortNum].modulo = 0;
  Ports[PortNum].ud_index_current[0]=0;
  Ports[PortNum].ud_index_current[1]=0;
  Ports[PortNum].ud_index_current[2]=0;

  /* The auxiliary timer is started */  
  (Ports[PortNum].htimcountrx).Instance->CNT = 0;
  SET_BIT((Ports[PortNum].htimcountrx).Instance ->EGR, TIM_EGR_UG);
  HAL_TIM_OC_Start_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum));
}


/**
  * @brief  PHY accomplishes the data receiving phase
  * @param  PortNum The port index
  * @retval None
  */
void PHY_HW_IF_RX_Stop(uint8_t PortNum)
{
  /* Within this function a check of the exed_flag variable is made.
   * The values associated to thye exit cases are:
   * 0 - Reset value
   * 1 - 
   * 2 - EOP detected
   * 3 - PHY_HW_IF_RX_Stop function is ongoing
   */

  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  
  if (ud->exed_flag == 3)
  {
    HAL_TIM_OC_Stop_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum));
    return;
  }

  uint8_t ud_ef = ud->exed_flag;
  ud->exed_flag = 3;

  /* stop the TIM counter for the selected port */
  SINGLE_TIM_OC_Stop_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum), RX_COUNTTIMCH_TIMIT(PortNum));

  /* Complete the parsing process */
  if (ud_ef == 0)
  {
    HW_IF_RX_CompleteParsingData(PortNum);
  }

  /* Stop DMA */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);

  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;

  /* Disable the TIM Update interrupt */
  HAL_TIM_Base_Stop_IT(&(Ports[PortNum].htimcountrx));

  /* stop the TIM counter for the selected port */
  Ports[PortNum].htimcountrx.Instance->CNT = 0;

  /* Callback core phy completed */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed(PortNum);
  }
}


/**
* @brief  Packet transmission has been accomplished
* @param  PortNum The port index
* @retval none
*/
void PHY_HW_IF_TX_Done(uint8_t PortNum)
{
  uint8_t dummyDR;
  uint8_t i = 0;

  /* Wait until FIFO is empty */
  if (Ports[PortNum].TxSpareBits == 0)
  {
    if (Ports[PortNum].Device_cut == Cut_1)
    {
      while (((((Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL)) >> SPI_SR_FTLVL_Pos) & 0x03) > 1);
    }
    else
    {
      while ( (Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL) != 0);  /* != 0x0800 */
    }
  }
  else
  {
    while (((((Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL)) >> SPI_SR_FTLVL_Pos) & 0x03) > 1)
    {
      __NOP();
    }
  }

  /* Wait for BUSY flag */
  while ( (Ports[PortNum].hspi.Instance->SR & SPI_SR_BSY) > 0);

  /* Act on TX spare bits */

  if (Ports[PortNum].Device_cut == Cut_1)
  {
    if (Ports[PortNum].TxSpareBits == 0)
    {
      Ports[PortNum].TxSpareBits = 7;
    }
    else
    {
      Ports[PortNum].TxSpareBits = (Ports[PortNum].TxSpareBits - 1);
    }
  }

  if (Ports[PortNum].Device_cut == Cut_1)
  {
    /* Complete transmission by adding spare bits */
    for(i=0; i<(Ports[PortNum].TxSpareBits); i++)
    {
      /* Wait for SPI CLK GPIO flag SET */
      while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
      /* Wait for SPI CLK GPIO flag RESET */
      while (!HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
    }
    while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
  }
  else  /* Cut_1_A */
  {
    for(i=0; i<(Ports[PortNum].TxSpareBits); i++)
    {
      /* Wait for SPI CLK GPIO flag RESET */
      while (!HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
      /* Wait for SPI CLK GPIO flag SET */
      while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
    }
  }

  /* Reset TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);

  /* Here the SPI has accomplished the transmission*/

  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);

  /* RX FIFO is cleaned */
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)&Ports[PortNum].hspi.Instance->DR;
    UNUSED(dummyDR);
  }

  /* SPI DMA is stopped */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);

  /* Check if BIST TX Done */
  if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
  {
    Ports[PortNum].State=HAL_USBPD_PORT_STATE_RESET;
    /* Evaluate callback*/
    if ((Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted != NULL) )
    {
      Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted(PortNum,USBPD_BIST_CARRIER_MODE2);
    }
  }
  else
  {
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  }

  /* Set the RX mode */
  STUSB16xx_HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);

  /* TX completed callback */
  if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum);
  }
}


/** @}*/ // End of PRIVATE_FUNCTIONS group


/* Inner functions -----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Inner_Functions USBPD DEVICE PHY HW IF Inner functions
  * @details Inner functions can be used at file level
  * @{
  */


/**
  * @brief  CRC init procedure
  * @retval None
  */
void HW_IF_CRC_Init(void)
{
  hcrc.Instance = CRC;

  hcrc.Init.DefaultPolynomialUse =   DEFAULT_POLYNOMIAL_ENABLE;              /* The default polynomial is used */
  hcrc.Init.DefaultInitValueUse =   DEFAULT_INIT_VALUE_ENABLE;              /* The default init value is used */
  hcrc.Init.InputDataInversionMode =   CRC_INPUTDATA_INVERSION_BYTE;           /* The input data are inverted by WORD */
  hcrc.Init.OutputDataInversionMode =   CRC_OUTPUTDATA_INVERSION_ENABLE;        /* The output data are Bit-reversed format */
  hcrc.InputDataFormat =           CRC_INPUTDATA_FORMAT_BYTES;             /* The input data are 32 bits lenght */

  HAL_CRC_Init(&hcrc);
}


/**
 * @brief  ADC init function
 * @retval None
 */
void HW_IF_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  uint8_t ch = 0;

  /* Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
  usbpdm1_hadc.Instance = HW_IF_ADC;

  usbpdm1_hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  usbpdm1_hadc.Init.Resolution = ADC_RESOLUTION_12B;
  usbpdm1_hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  usbpdm1_hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  usbpdm1_hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  usbpdm1_hadc.Init.LowPowerAutoWait = DISABLE;
  usbpdm1_hadc.Init.LowPowerAutoPowerOff = DISABLE;
  usbpdm1_hadc.Init.ContinuousConvMode = ENABLE;
  usbpdm1_hadc.Init.DiscontinuousConvMode = DISABLE;
  usbpdm1_hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  usbpdm1_hadc.Init.DMAContinuousRequests = ENABLE;
  usbpdm1_hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  
  HAL_ADC_Init(&usbpdm1_hadc);
  
  /* Configuration of the selected ADC regular channel to be converted */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    sConfig.Channel = USBPD_ADCs[ch].ADCCH;
    HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  }
}


/**
 * @brief  UnwrapData structure init function
 * @param  PortNum The port index
 * @retval None
 */
void HW_IF_UnwrapData_Init(uint8_t PortNum)
{
  /* Decoding variables */
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);

  /* Init the decoding variables */
  ud->exed_flag =       0;
  ud->preamble =        0;
  ud->dataindex =       0;
  ud->dataoffset =      0;
  ud->index =           2;      /* It discards first two bytes */
  ud->offset =          0;

  preamble_offset = 0;
  preamble_index = 0;
  preamble_counter++;
}


/**
 * @brief  It completes data parsing
 * @param  PortNum The port index
 * @retval None
 */
static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum)
{
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pRxBuffPtr;
  uint32_t temp_data = 0;

  uint16_t lastindex = (RX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR);

  /* If callback is not available skip the accumulation phase */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate == NULL)
  {
    __NOP();
    return;
  }
  while(ud->index <= lastindex)
  {
    memcpy(&temp_data, &pbuff_in[ud->index], 3);
    temp_data = (temp_data >> ud->offset) & 0x3FF;
    ud->index += ud->offset <= 5 ? 1 : 2;
    ud->offset = (ud->offset + 2) & 7;

    /* Callback core phy accumulate */
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, temp_data);

    /* EOP detecting */
    if ((temp_data & 0x1F) == 0x0D)
    {
      break;
    }
  }
}


/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */


/**
* @brief  Stops the TIM Output Compare signal generation in interrupt mode.
  * @param  htim TIM Output Compare handle
  * @param  Channel TIM Channel to be disabled
*          This parameter can be one of the following values:
*            @arg TIM_CHANNEL_1: TIM Channel 1 selected
*            @arg TIM_CHANNEL_2: TIM Channel 2 selected
*            @arg TIM_CHANNEL_3: TIM Channel 3 selected
*            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  tim_it TIM IT
* @retval HAL status
*/
static inline void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Disable the TIM Capture/Compare 1 interrupt */
  __HAL_TIM_DISABLE_IT(htim, tim_it);

  /* Disable the Output compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);

  if(IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
  {
    /* Disable the Main Ouput */
    htim->Instance->BDTR &= ~(TIM_BDTR_MOE);
  }
  /* Disable the Peripheral */
  htim->Instance->BDTR &= ~(TIM_BDTR_MOE);
}


/** @}*/ // End of INNER_FUNCTIONS group


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
