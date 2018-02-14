/**
  ******************************************************************************
  * @file    usbpd_hw_if.c
  * @author  System Lab
  * @brief   This file contains power interface control functions.
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
#include "bmc.h"
#include "string.h"
#include "usbpd_cad.h"
#include "usbpd_timersserver.h"
#include "usbpd_porthandle.h"
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"

#ifdef MB1303
#include "p-nucleo-usb002.h"
#else
#include "STUSB16xx_EVAL.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*FLR: moved to .h */
//#define DMA_TIME_ELAPSED        60 //us
//#define DMA_TIME_COUNT_COMPARE  10 //us
#define DMA_TIME_TASK           45 //us
#define NumberOfCodedBit    80 /*!< Number of bit decoded each time by COUNT_TIM  */
#define BITOFFWORD          0   /*!< if bitoff is 2 no shift inside word; if 0 MS 2 bit are left 0 to use 30 bit word*/

/* Private macro -------------------------------------------------------------*/
/* Note (GN): TX_BUFFER_LEN to be checked */
#define 	TX_BUFFER_SIZE	56 //( TX_BUFFER_LEN * 4 )
#define 	TX_PREAMBLE 0xAA                        /* Preamble byte 10101010 = 0xAA */
#define 	TX_PREAMBLE_SIZE 8                      /* Amount of repeated preamble bytes */

/* Private variables ---------------------------------------------------------*/
uint8_t         RXBuffer0[PHY_MAX_RAW_BYTE_SIZE]; /*!< Buffer for raw data received           */
/* Note (GN): A define has to be used */
uint32_t        RXData0[22];                    /*!< Buffer for 5b decoded data received    */
uint8_t         TXBuffer0[TX_BUFFER_SIZE];      /*!< Buffer for data to be transmitted      */
#if (USBPD_PORT_COUNT == 2)
uint8_t         RXBuffer1[PHY_MAX_RAW_BYTE_SIZE];
uint32_t        RXData1[22];
uint8_t	        TXBuffer1[TX_BUFFER_SIZE];
#endif

CRC_HandleTypeDef   hcrc;                   /*!< Handle of CRC peripheral */
ADC_HandleTypeDef   usbpdm1_hadc;           /*!< Handle of ADC peripheral */
DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */

/* Variable containing ADC conversions results */
uint32_t   ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Array of ADC CH pins used by @ref STUSB16xx_EVAL */
USBPD_BSP_GPIOPins_TypeDef USBPD_ADCs[USBPD_ADCn] =
{
  USBPD_BSP_ADC(GPIOC,5,ADC_CHANNEL_15)
};


extern STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
void HW_IF_UnwrapData_Init(uint8_t PortNum);

/* Common peripheral Init function*/
void HW_IF_CRC_Init(void);
void HW_IF_ADC_Init(void);


/* Init of decoding support variable */
HAL_StatusTypeDef check_bus_voltage(uint8_t PortNum);


/* Strict Private functions ---------------------------------------------------------*/
static inline void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum);


/**
  * @brief  Global hardware initialization 
  * @param  None
  * @retval None
  */ 
void USBPD_HW_IF_GlobalHwInit(void)
{
  /* Configure the ADCx peripheral */
//  HW_IF_ADC_Init();
  
  /* Run the ADC calibration */
//  HAL_ADCEx_Calibration_Start(&usbpdm1_hadc);
  
  /* Start ADC conversion on regular group with transfer by DMA */
//  HAL_ADC_Start_DMA(&usbpdm1_hadc, ADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE);
  
  HW_IF_CRC_Init();
  HW_IF_PWR_DigitalGPIO_Init();

  /* used for the PRL/PE timing */
  USBPD_TIM_Init();
}






void HW_IF_CRC_Init(void)
{
  hcrc.Instance = CRC;
  
  hcrc.Init.DefaultPolynomialUse = 	DEFAULT_POLYNOMIAL_ENABLE;  	/* The default polynomial is used */
  hcrc.Init.DefaultInitValueUse = 	DEFAULT_INIT_VALUE_ENABLE;      /* The default init value is used */
  hcrc.Init.InputDataInversionMode = 	CRC_INPUTDATA_INVERSION_BYTE;   /* The input data are inverted by WORD */
  hcrc.Init.OutputDataInversionMode =   CRC_OUTPUTDATA_INVERSION_ENABLE;/* The output data are Bit-reversed format */
  hcrc.InputDataFormat = 	        CRC_INPUTDATA_FORMAT_BYTES;     /* The input data are 32 bits lenght */
  
  HAL_CRC_Init(&hcrc);
}


uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint8_t len)
{
  uint32_t crc;
  
  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)pBuffer, len);
  crc ^= 0xFFFFFFFF;
  return crc;
}



/**
 * @brief  ADC init function
 * @param  None
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
  
  /**Configure for the selected ADC regular channel to be converted. */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  
  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    sConfig.Channel = USBPD_ADCs[ch].ADCCH;
    HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  }
  
}


/**
 * @brief  Initialize the ADC Analog GPIOs.
 * @param  None
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
 * @brief  Deinitialize the ADC Analog GPIOs.
 * @param  None
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
  * @brief  Initialize the ADC DMA
  * @param  None
  * @retval None
  */
void PHY_HW_IF_ADCDMA_Init(void)
{
  /* Configure DMA parameters */
  DmaHandle.Instance = ADCx_DMA;
  
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;       /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_LOW;
  
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);
  
  /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(&usbpdm1_hadc, DMA_Handle, DmaHandle);
  
  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(ADC1_COMP_IRQn, USBPD_LOWEST_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}


/**
  * @brief  Deinitialize the ADC DMA
  * @param  None
  * @retval None
  */
void PHY_HW_IF_ADCDMA_DeInit(void)
{
  /* De-Initialize the DMA associated to the peripheral */
  if(usbpdm1_hadc.DMA_Handle != NULL)
  {
	HAL_DMA_DeInit(usbpdm1_hadc.DMA_Handle);
  }
}



void USBPD_HW_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef cc)
{
/* Functionality supported by STUSB1602 */
}



/**
  * @brief  PHY_HW_IF_RX_Start
  * @param  PortNum: port index
  * @retval void
  */ 
void PHY_HW_IF_RX_Start(uint8_t PortNum)
{
  /* Callback core phy reset */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset != NULL) 
  {
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset(PortNum);
  }
  
  /* For debug purposes */
//  for (uint8_t i=0;i<10;i++)   pData[i] = 0;
//  pDataIndex = 0;
  
  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_RX;
  
  /* Variables for decoding stage are initalized */
  HW_IF_UnwrapData_Init(PortNum);
  
    HAL_SPI_DMAStop(&Ports[PortNum].hspi);
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);

  /* Clean SPI DR before to receive */
  SPI_HandleTypeDef * local_hspi = &Ports[PortNum].hspi;
  __IO uint32_t a = 0x0;
  while(local_hspi->Instance->SR & 0x01)
  {
    a = local_hspi->Instance->DR;
  }
  UNUSED(a);
  
  memset(Ports[PortNum].pRxBuffPtr, 0x11, PHY_MAX_RAW_BYTE_SIZE);

  /* Start DMA receiving */
  HAL_SPI_Receive_DMA(&Ports[PortNum].hspi, (uint8_t*)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_BYTE_SIZE); /*FLR: data length to be modified */

  /* The auxiliary timer is started */  
  (Ports[PortNum].htimcountrx).Instance->CNT = 0;
  HAL_TIM_OC_Start_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum));
}


/**
  * @brief  PHY_HW_IF_RX_Start
  * @param  PortNum: port index
  * @retval void
  */ 
void PHY_HW_IF_RX_Stop(uint8_t PortNum)
{
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  if (ud->exed_flag == 3)
    return;

  uint8_t ud_ef = ud->exed_flag;
  ud->exed_flag = 3;
  
  /* stop the TIM counter for the selected port */    
  SINGLE_TIM_OC_Stop_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum), RX_COUNTTIMCH_TIMIT(PortNum));

  /* Complete parsing process */
  if (ud_ef == 0)
  {
    HW_IF_RX_CompleteParsingData(PortNum);
  }

  /* Stop DMA */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
  __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);

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
* @brief  SendBuffer
* @param  PortNum: port index
* @param  *pBuffer:
* @param  Bitsize:
* @retval HAL_StatusTypeDef
*/ 
HAL_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize)
{
  /* Check if the port is yet receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
    return HAL_BUSY;
  
  uint8_t *pTxBuffer = (uint8_t *)Ports[PortNum].pTxBuffPtr;
  uint16_t size = DIV_ROUND_UP(Bitsize, 8)+TX_PREAMBLE_SIZE;
  
  memset((uint8_t *)pTxBuffer, 0x00, TX_BUFFER_SIZE);
  memset((uint8_t *)pTxBuffer, TX_PREAMBLE, TX_PREAMBLE_SIZE);                          /* preamble is added */
  memcpy((uint8_t *)(pTxBuffer+TX_PREAMBLE_SIZE), pBuffer, (size-TX_PREAMBLE_SIZE));    /* data are added */

  /* Spare clock cicles at the end of transmission are calculated */
  Ports[PortNum].TxSpareBits = (Bitsize % 8);

  /* Send packet to STUSB1602 */
  HAL_StatusTypeDef ret = HAL_OK;
  ret = STUSB16xx_HW_IF_Send_Packet(PortNum, pTxBuffer, size);
  return ret;
}



uint8_t dummyDR;
/**
* @brief  Packet transmission has been accomplished
* @param  PortNum: port index
* @retval none
*/ 
void PHY_HW_IF_TX_Done(uint8_t PortNum)
{
  uint8_t i = 0;

  /* Wait for FIFO empty */
  
  if (Ports[PortNum].TxSpareBits == 0)
  {
    
    if (Ports[PortNum].Device_cut == Cut_1) while (((((Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL)) >> SPI_SR_FTLVL_Pos) & 0x03) > 1);
    else while ( (Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL) != 0);  /* != 0x0800 */
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
  {  /* STUSB1602_CUT_1_2 */
    if (Ports[PortNum].TxSpareBits == 0)
    {
      Ports[PortNum].TxSpareBits = 7;
    }
    else
    {      
      Ports[PortNum].TxSpareBits = (Ports[PortNum].TxSpareBits - 1);
    }
  }
         
    /* STUSB1602_CUT_1_2 */
    if (Ports[PortNum].Device_cut == Cut_1)
      {  
        /* Complete transmission with spare bts */
  for(i=0; i<(Ports[PortNum].TxSpareBits); i++)
          {
            /* Wait for SPI CLK GPIO flag SET */
            while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
            /* Wait for SPI CLK GPIO flag RESET */
            while (!HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
          }
          while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)));
      }
    else  /* STUSB1602_CUT_1_3 */
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
  
  
  /* Here the SPI has completed the transmission*/
  
  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);  
  
  /* RX FIFO is cleaned */
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)Ports[PortNum].hspi.Instance->DR;
  }
  
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);
  
  /* DMA Abort to execute the DMA Deinit */
  HAL_DMA_Abort(&Ports[PortNum].hdmatx);
  
  /* Check if BIST TX Done */
  if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
  {
    // Add callback here
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



  /* Note (GN): BIST has to be implemented */
  /* Note (FLR): old BIST modified */

HAL_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  
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
  /* Set TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
  
  ret = HAL_OK;
  return ret;
}



uint8_t preamble_offset;
uint8_t preamble_index;
uint32_t preamble_counter = 0;


void HW_IF_UnwrapData_Init(uint8_t PortNum)
{
  /* Decoding variables */
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);

  /* Init the decoding variables */
  ud->exed_flag =       0;
  ud->preamble =        0;
  ud->dataindex =       0;
  ud->dataoffset =      0;
  ud->index =           2;      /* Discarding first two bytes */
  ud->offset =          0;
  
  preamble_offset = 0;
  preamble_index = 0;
  preamble_counter++;
}

uint8_t ud_index_current[4] = {0, 0,0}; // N Ballot was 4
uint8_t modulo=0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  
  uint8_t PortNum, r, prev_bit, curr_bit;
  uint32_t temp_data = 0;
  
#if (USBPD_PORT_COUNT==1)
  PortNum = 0;
#elif (USBPD_PORT_COUNT==2)           
  PortNum = ( (htim->Instance == RX_COUNTTIM(0)) ? 0 : 1);
#endif /* RX TIMER IDENTIFIED */
  
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pRxBuffPtr;
  uint32_t tim_count = Ports[PortNum].htimcountrx.Instance->CNT;
  //  uint32_t dma_count = Ports[PortNum].hdmarx.Instance->CNDTR;
  
  
  
  while (
         (ud->exed_flag == 0) &&
         ((PHY_MAX_RAW_BYTE_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR)>(ud->index + 2)) &&  
         (Ports[PortNum].htimcountrx.Instance->CNT < tim_count+20)
        )
  {
    
    if (pbuff_in[ud->index]==0xFF)
    {
      __NOP();
    }

    if (!ud->preamble)
    {
      /* Search end of preamble */
      r = pbuff_in[ud->index]^0xAA;
      if (r == 0x00 || r == 0xFF)
      {
        /* Preamble */
        ud->index++;
      }
      else
      {
        prev_bit = (pbuff_in[ud->index-1]>>7) & 0x01;
        while (ud->offset < 8)
        {
          curr_bit = (pbuff_in[ud->index]>>ud->offset) & 0x01;
          if (prev_bit == curr_bit)
          {
            /* Preamble identified. Offset index is the start of data after preamble */
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
      
      __NOP();
      memcpy(&temp_data, &pbuff_in[ud->index], 3);
      temp_data = (temp_data >> ud->offset) & 0x3FF;
      ud->index += ud->offset <= 5 ? 1 : 2;//((offset + 1 )>>2)+1
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
ud_index_current[modulo] = ud->index ;
	modulo++;
	modulo = modulo%3;
        if (
		(ud_index_current[modulo] == ud_index_current[(modulo+1)%3]) && 
		(ud_index_current[(modulo+1)%3] == ud_index_current[(modulo+2)%3]) &&
        (ud->index > 5) 
		)
	{ 
		ud->exed_flag = 3; 
	}
  
  /* */
  if (ud->exed_flag == 2)
  {
    PHY_HW_IF_RX_Stop(PortNum);
  }
}


static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum)
{
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pRxBuffPtr;
  uint32_t temp_data = 0;
  
  uint16_t lastindex = (PHY_MAX_RAW_BYTE_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR);
  
  /* If callback is not available skip the accumulation phase */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate == NULL)
    return;
  
  while(ud->index <= lastindex)
  {
    memcpy(&temp_data, &pbuff_in[ud->index], 3);
    temp_data = (temp_data >> ud->offset) & 0x3FF;
    ud->index += ud->offset <= 5 ? 1 : 2;
    ud->offset = (ud->offset + 2) & 7;
    
    /* Callback core phy accumulate */
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, temp_data);
    
    /* Note (GN): Following two raws have to be removed */
//    pData[pDataIndex] = temp_data;
//    pDataIndex++;
    
    /* EOP detecting */
    if ((temp_data & 0x1F) == 0x0D)
    {
      break;
    }
  }
}



/* -------------------------------------------------------------------------- */
/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */


/**
* @brief  Stops the TIM Output Compare signal generation in interrupt mode.
* @param  htim : TIM Output Compare handle
* @param  Channel : TIM Channel to be disabled
*          This parameter can be one of the following values:
*            @arg TIM_CHANNEL_1: TIM Channel 1 selected
*            @arg TIM_CHANNEL_2: TIM Channel 2 selected
*            @arg TIM_CHANNEL_3: TIM Channel 3 selected
*            @arg TIM_CHANNEL_4: TIM Channel 4 selected
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

//static inline void SINGLE_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
//{
//  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_LDMATX);
//
//  /* Enable the Tx DMA channel */
//  HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)pData, (uint32_t)&hspi->Instance->DR, Size);
//  /* Enable SPI peripheral */
//  __HAL_SPI_ENABLE(hspi);
//  /* Enable Tx DMA Request */
//  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
//}

/**
* @brief  Stops the TIM Input Capture measurement in interrupt mode.
* @param  htim : TIM handle
* @param  Channel : TIM Channels to be disabled
*          This parameter can be one of the following values:
*            @arg TIM_CHANNEL_1: TIM Channel 1 selected
*            @arg TIM_CHANNEL_2: TIM Channel 2 selected
*            @arg TIM_CHANNEL_3: TIM Channel 3 selected
*            @arg TIM_CHANNEL_4: TIM Channel 4 selected
* @retval HAL status
*/
static inline void SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
  __HAL_TIM_DISABLE_IT(htim, tim_it);
  /* Disable the Input Capture channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
  /* Disable the Peripheral */
  htim->Instance->CR1 &= ~(TIM_CR1_CEN);
}

void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  SINGLE_TIM_IC_Stop_IT(htim,Channel,tim_it);
}

void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode)
{
//  /* Reset the BIST index*/
//  Ports[PortNum].BIST_index= 0;
//  /* Reset the Bypass Bus Idle Flag */
//  Ports[PortNum].BusIdleFlg = 0;
//  /* Initialize State */
//  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
//  RX_Init_Hvar(PortNum);  
}

void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(gpio.GPIOx, gpio.GPIO_Pin, PinState);
}
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Sets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_SET);
}

void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Resets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_RESET);
}

void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Toggle the pin */
  HAL_GPIO_TogglePin(gpio.GPIOx, gpio.GPIO_Pin);
}
/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
