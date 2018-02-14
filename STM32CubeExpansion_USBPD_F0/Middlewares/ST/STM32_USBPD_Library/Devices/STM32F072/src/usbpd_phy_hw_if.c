/**
  ******************************************************************************
  * @file    usbpd_hw_if.c
  * @author  System Lab
  * @brief   This file contains power interface control functions.
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
#include "bmc.h"
#include "string.h"
#include "usbpd_timersserver.h"
#include "p-nucleo-usb001.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_comp.h"

/* Private typedef -----------------------------------------------------------*/

/**
  * @brief USB PD Board Version MB1257B/C
  */
typedef enum
{
  USBPD_BOARDVERSION_INVALID   = 0,
  USBPD_BOARDVERSION_MB1257_C  = 1,
  USBPD_BOARDVERSION_MB1257_B  = 2,
} USBPD_BoardVersionTypeDef;

/* Private define ------------------------------------------------------------*/
#define DMA_TIME_ELAPSED        60 /* us */
#define DMA_TIME_TASK           45 /* us */
#define DMA_TIME_COUNT_COMPARE  8  /* us */
#define DMA_TIME_DURATION       5 /* us */
#define DMA_TIME_THRESHOLD1    ((DMA_TIME_ELAPSED)-(2*DMA_TIME_DURATION)) //us 50
#define DMA_TIME_THRESHOLD2    ((DMA_TIME_ELAPSED)-(DMA_TIME_DURATION))   //us 55
#define NumberOfCodedBit        80 /*!< Number of bit decoded each time by COUNT_TIM  */
#define BITOFFWORD              0  /*!< if bitoff is 2 no shift inside word; if 0 MS 2 bit are left 0 to use 30 bit word*/

#define PARAM_RX_OFFSET 5
/* Private macro -------------------------------------------------------------*/
#define TX_BUFFER_SIZE          ( TX_BUFFER_LEN * 4 )
#define TX_PREAMBLE_BMC_CODED 0xB4              /* (BMC_Coding[1][0xA]) //1010 => 10110100 = 0xB4 with 1 as previous */
#define TX_PREAMBLE_SIZE 16                     /* bmc bytes */
#define RX_BUFFER_SIZE          22

/* Private variables ---------------------------------------------------------*/
uint8_t     RXBuffer0[PHY_MAX_RAW_SIZE];    /*!< Buffer for raw data received           */
uint32_t    RXData0[RX_BUFFER_SIZE];        /*!< Buffer for 5b decoded data received    */
uint32_t    TXBuffer0[TX_BUFFER_LEN];       /*!< Buffer for data to be transmitted      */
#if (USBPD_PORT_COUNT == 2)
uint8_t     RXBuffer1[PHY_MAX_RAW_SIZE];
uint32_t    RXData1[RX_BUFFER_SIZE];
uint32_t    TXBuffer1[TX_BUFFER_LEN];
#endif

static USBPD_BoardVersionTypeDef BoardVersion;

CRC_HandleTypeDef   hcrc;                   /*!< Handle of CRC peripheral */
ADC_HandleTypeDef   usbpdm1_hadc;           /*!< Handle of ADC peripheral */
DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */

/* Variable containing ADC conversions results */
uint32_t   ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Handle for the ports inside @ref USBPD_HW_IF*/
USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/* Array of GPIO used by @ref P-NUCLEO-USB001 */
const USBPD_BSP_GPIOPins_TypeDef USBPDM1_GPIOs[USBPDM1_GPIOn] =
{
  USBPD_BSP_PIN(GPIOB,2),
  USBPD_BSP_PIN(GPIOB,12),
  USBPD_BSP_PIN(GPIOB,9),
  USBPD_BSP_PIN(GPIOC,6),
  USBPD_BSP_PIN(GPIOC,3),
  USBPD_BSP_PIN(GPIOA,8),
  USBPD_BSP_PIN(GPIOC,7),
  USBPD_BSP_PIN(GPIOC,8),
  USBPD_BSP_PIN(GPIOA,15),
  USBPD_BSP_PIN(GPIOB,5),
  USBPD_BSP_PIN(GPIOB,8),
  USBPD_BSP_PIN(GPIOD,2),
  USBPD_BSP_PIN(GPIOC,14),
#ifdef P_NUCLEO_USB001_GPIO13
  USBPD_BSP_PIN(GPIOC,15),
#endif
  USBPD_BSP_PIN(GPIOB,0),
#ifdef P_NUCLEO_USB001_GPIO15
  USBPD_BSP_PIN(GPIOF,1),
#endif
#ifndef P_NUCLEO_USB001_USE_USB2
  USBPD_BSP_PIN(GPIOA,11),
  USBPD_BSP_PIN(GPIOA,12)
#endif
};

/* Array of power selection pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_POWSELs[USBPDM1_POWSELn] =
{
  USBPD_BSP_PIN(GPIOB,7),
  USBPD_BSP_PIN(GPIOB,6),
  USBPD_BSP_PIN(GPIOC,1),
  USBPD_BSP_PIN(GPIOC,9),
};

/* Array of ADC CH pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_ADCs[USBPDM1_ADCn] =
{
#if (USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1)  
  USBPD_BSP_ADC(GPIOC,5,ADC_CHANNEL_15),
  USBPD_BSP_ADC(GPIOA,7,ADC_CHANNEL_7),
#else
  USBPD_BSP_ADC(GPIOC,4,ADC_CHANNEL_14),
  USBPD_BSP_ADC(GPIOA,3,ADC_CHANNEL_3),
#endif
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOC,5,ADC_CHANNEL_15),
  USBPD_BSP_ADC(GPIOA,7,ADC_CHANNEL_7),
#endif
  USBPD_BSP_ADC(GPIOC,0,ADC_CHANNEL_10)
};

/* Array of RXD pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_RXD[USBPDM1_RXDn] =
{
#if (USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1)
  USBPD_BSP_ADC(GPIOA,2,ADC_CHANNEL_2), //P1.CC1
  USBPD_BSP_ADC(GPIOA,4,ADC_CHANNEL_4), //P1.CC2
#else
  USBPD_BSP_ADC(GPIOA,0,ADC_CHANNEL_0), //P0.CC1
  USBPD_BSP_ADC(GPIOA,5,ADC_CHANNEL_5), //P0.CC2
#endif
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOA,2,ADC_CHANNEL_2), //P1.CC1
  USBPD_BSP_ADC(GPIOA,4,ADC_CHANNEL_4), //P1.CC2
#endif
};

/* RXREF Pin used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_RXREF = USBPD_BSP_ADC(GPIOA,1,ADC_CHANNEL_1);

/* Private function prototypes -----------------------------------------------*/

/* Common peripheral Init function*/
void USBPDM1_CRC_Init(void);
void USBPDM1_ADC_Init(void);
USBPD_BoardVersionTypeDef USBPD_HW_IF_GetBoardVersion(void); 

/* Port related Init functions */
void USBPDM1_DMA_Init(uint8_t PortNum);
void USBPDM1_COMP_SetCC(uint8_t PortNum, CCxPin_TypeDef ccx);
void USBPDM1_SPI_Init(uint8_t PortNum);
void USBPDM1_TX_TIM_Init(uint8_t PortNum);
void USBPDM1_RX_TIM_Init(uint8_t PortNum);
void USBPDM1_COMP_Init(uint8_t PortNum);
void USBPDM1_COUNTTIM_Init(uint8_t PortNum);

/* Pinout management functions */
__STATIC_INLINE void USBPDM1_DeInitRXD(uint8_t PortNum,CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_ReInitRXD(uint8_t PortNum,CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_SPI_Reset_TX_CC(uint8_t PortNum, CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_SetRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role);

/* Optimized functions */
__STATIC_INLINE void SINGLE_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin, uint32_t position );
__STATIC_INLINE void SINGLE_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init, uint32_t position);
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
__STATIC_INLINE void SINGLE_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
__STATIC_INLINE void SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);

/* Init of decoding support variable */
__STATIC_INLINE void HW_IF_DecodingTask(uint8_t PortNum, uint8_t Phase);
USBPD_StatusTypeDef HW_IF_CheckBusIdle(uint8_t PortNum);

/* Private functions ---------------------------------------------------------*/

USBPD_BoardVersionTypeDef USBPD_HW_IF_GetBoardVersion()
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  USBPD_BoardVersionTypeDef Version;

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  /* TX CC1 */
  GPIO_InitStruct.Pin = TX_CC1_PIN(0);
  HAL_GPIO_Init(TX_CC1_GPIOPORT(0), &GPIO_InitStruct);
  
  /* CC1 Enable on Port 0 */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC1_P0].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC1_P0].GPIOx, &GPIO_InitStruct);
  
  /* CC2 Enable on Port 0 */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC2_P0].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC2_P0].GPIOx, &GPIO_InitStruct);
  
  /* Rd resistance on Port 0 */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRD_P0].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[HRD_P0].GPIOx, &GPIO_InitStruct);    
  
  /* Rp resistance on Port 0 */
  GPIO_InitStruct.Pin   = USBPDM1_GPIOs[HRP_P0].GPIO_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  HAL_GPIO_Init(USBPDM1_GPIOs[HRP_P0].GPIOx, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(USBPDM1_GPIOs[ENCC1_P0].GPIOx, USBPDM1_GPIOs[ENCC1_P0].GPIO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USBPDM1_GPIOs[ENCC2_P0].GPIOx, USBPDM1_GPIOs[ENCC2_P0].GPIO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USBPDM1_GPIOs[HRD_P0].GPIOx, USBPDM1_GPIOs[HRD_P0].GPIO_Pin, GPIO_PIN_RESET);    
  HAL_GPIO_WritePin(TX_CC1_GPIOPORT(0), TX_CC1_PIN(0), GPIO_PIN_SET);

  USBPD_TIM_Start(TIM_PORT0_CRC, 2000);
  while(USBPD_TIM_IsExpired(TIM_PORT0_CRC) == 0)
  {
  }
  
  if (HAL_GPIO_ReadPin(USBPDM1_GPIOs[HRP_P0].GPIOx, USBPDM1_GPIOs[HRP_P0].GPIO_Pin) == GPIO_PIN_SET)
  {
    Version = USBPD_BOARDVERSION_MB1257_B;
  }
  else
  {
    Version = USBPD_BOARDVERSION_MB1257_C;
  }    
  
  HAL_GPIO_DeInit(USBPDM1_GPIOs[HRP_P0].GPIOx, USBPDM1_GPIOs[HRP_P0].GPIO_Pin);
  HAL_GPIO_DeInit(TX_CC1_GPIOPORT(0), TX_CC1_PIN(0));
  HAL_GPIO_DeInit(USBPDM1_GPIOs[ENCC1_P0].GPIOx, USBPDM1_GPIOs[ENCC1_P0].GPIO_Pin);
  HAL_GPIO_DeInit(USBPDM1_GPIOs[ENCC2_P0].GPIOx, USBPDM1_GPIOs[ENCC2_P0].GPIO_Pin);
  HAL_GPIO_DeInit(USBPDM1_GPIOs[HRD_P0].GPIOx, USBPDM1_GPIOs[HRD_P0].GPIO_Pin);    
  
  return Version;
}

void USBPDM1_DigitalGPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint32_t gpio=0;
  
  for(gpio=0;gpio<USBPDM1_GPIOn;gpio++)
  {
    /* Configure the GPIO pin */
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[gpio].GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(USBPDM1_GPIOs[gpio].GPIOx, &GPIO_InitStruct);
  }

  for(gpio=0;gpio<USBPDM1_POWSELn;gpio++)
  {
    /* Configure the powsels pin */
    GPIO_InitStruct.Pin = USBPDM1_POWSELs[gpio].GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(USBPDM1_POWSELs[gpio].GPIOx, &GPIO_InitStruct);
    
    /* Turn the pin off */
    // USBPDM1_GPIO_Off((USBPDM1_GPIO_TypeDef)gpio);
  }
  
  /* Initialize the DRP pin*/
  GPIO_InitStruct.Pin = DRP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRP_PORT,&GPIO_InitStruct);
  HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_SET);
  
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
  /* ENCC1 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC1_P1].GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC1_P1].GPIOx, &GPIO_InitStruct);
  /* ENCC2 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC2_P1].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC2_P1].GPIOx, &GPIO_InitStruct);
#else
  /* ENCC1 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC1_P0].GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC1_P0].GPIOx, &GPIO_InitStruct);
  /* ENCC2 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC2_P0].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC2_P0].GPIOx, &GPIO_InitStruct);
#endif
#if (USBPD_PORT_COUNT == 2)
  /* ENCC1 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC1_P1].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC1_P1].GPIOx, &GPIO_InitStruct);
  /* ENCC2 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC2_P1].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC2_P1].GPIOx, &GPIO_InitStruct);
#endif
}

void USBPDM1_GPIO_On(USBPDM1_GPIO_TypeDef gpio)
{
  /* Sets the pin */
  HAL_GPIO_WritePin(USBPDM1_GPIOs[gpio].GPIOx, USBPDM1_GPIOs[gpio].GPIO_Pin, GPIO_PIN_SET);
}

void USBPDM1_GPIO_Off(USBPDM1_GPIO_TypeDef gpio)
{
  /* Resets the pin */
  HAL_GPIO_WritePin(USBPDM1_GPIOs[gpio].GPIOx, USBPDM1_GPIOs[gpio].GPIO_Pin, GPIO_PIN_RESET);
}

void USBPDM1_GPIO_Toggle(USBPDM1_GPIO_TypeDef gpio)
{
  /* Toggle the pin */
  HAL_GPIO_TogglePin(USBPDM1_GPIOs[gpio].GPIOx, USBPDM1_GPIOs[gpio].GPIO_Pin);
}

void USBPDM1_Enable_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  switch (cc)
  {
  case CC1:
    USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
    break;
  case CC2:
    USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
    break;
  case CCNONE:
  default:
    break;
  }
}

void USBPD_HW_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef cc)
{
  switch (cc)
  {
    /* Turns on the VConn on the selected CC */
  case CC1:
    USBPDM1_GPIO_Off(ENCC1_PIN(PortNum));
    break;
  case CC2:
    USBPDM1_GPIO_Off(ENCC2_PIN(PortNum));
    break;
  case CCNONE:
    /* Turns off the VConn on all CC */
    USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
    USBPDM1_GPIO_Off(ENCC1_PIN(PortNum));
    USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
    
    USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
    USBPDM1_GPIO_Off(ENCC2_PIN(PortNum));
    USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
    break;
  }
}

void USBPDM1_SPI_Set_TX_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  switch (cc)
  {
    /* Switch Pins to enable transmission */
  case CC1:
    GPIO_InitStruct.Alternate = TX_CC1_SPI_GPIOAF(PortNum);
    GPIO_InitStruct.Pin = TX_CC1_PIN(PortNum);
    SINGLE_GPIO_Init(TX_CC1_GPIOPORT(PortNum), &GPIO_InitStruct, TX_CC1_PIN_POSITION(PortNum));
    break;
  case CC2:
    GPIO_InitStruct.Alternate = TX_CC2_SPI_GPIOAF(PortNum);
    GPIO_InitStruct.Pin = TX_CC2_PIN(PortNum);
    SINGLE_GPIO_Init(TX_CC2_GPIOPORT(PortNum), &GPIO_InitStruct, TX_CC2_PIN_POSITION(PortNum));
    break;
  case CCNONE:
    /* In this case deinits all pins*/
    HAL_GPIO_DeInit(TX_CC1_GPIOPORT(PortNum),TX_CC1_PIN(PortNum));
    HAL_GPIO_DeInit(TX_CC2_GPIOPORT(PortNum),TX_CC2_PIN(PortNum));
    break;
  }
}

__STATIC_INLINE void USBPDM1_SPI_Reset_TX_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  switch (cc)
  {
    /* Switch Pins to digital input to disable transmission */
  case CC1:
    GPIO_InitStruct.Pin = TX_CC1_PIN(PortNum);
    SINGLE_GPIO_Init(TX_CC1_GPIOPORT(PortNum), &GPIO_InitStruct, TX_CC1_PIN_POSITION(PortNum));
    break;
  case CC2:
    GPIO_InitStruct.Pin = TX_CC2_PIN(PortNum);
    SINGLE_GPIO_Init(TX_CC2_GPIOPORT(PortNum), &GPIO_InitStruct, TX_CC2_PIN_POSITION(PortNum));
    break;
  case CCNONE:
    /* In this case deinits all pins*/
    HAL_GPIO_DeInit(TX_CC1_GPIOPORT(PortNum),TX_CC1_PIN(PortNum));
    HAL_GPIO_DeInit(TX_CC2_GPIOPORT(PortNum),TX_CC2_PIN(PortNum));
    break;
  }
}

void USBPDM1_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  uint8_t ch = 0;
  
  /* Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
  usbpdm1_hadc.Instance = P_NUCLEO_USB001_ADC;
  
  usbpdm1_hadc.Init.ClockPrescaler              = ADC_CLOCK_SYNC_PCLK_DIV4;
  usbpdm1_hadc.Init.Resolution                  = ADC_RESOLUTION_12B;
  usbpdm1_hadc.Init.DataAlign                   = ADC_DATAALIGN_RIGHT;
  usbpdm1_hadc.Init.ScanConvMode                = ADC_SCAN_DIRECTION_FORWARD;
  usbpdm1_hadc.Init.EOCSelection                = ADC_EOC_SINGLE_CONV;
  usbpdm1_hadc.Init.LowPowerAutoWait            = DISABLE;
  usbpdm1_hadc.Init.LowPowerAutoPowerOff        = DISABLE;
  usbpdm1_hadc.Init.ContinuousConvMode          = ENABLE;
  usbpdm1_hadc.Init.DiscontinuousConvMode       = DISABLE;
  usbpdm1_hadc.Init.ExternalTrigConvEdge        = ADC_EXTERNALTRIGCONVEDGE_NONE;
  usbpdm1_hadc.Init.DMAContinuousRequests       = ENABLE;
  usbpdm1_hadc.Init.Overrun                     = ADC_OVR_DATA_PRESERVED;
  
  HAL_ADC_Init(&usbpdm1_hadc);
  
  /**Configure for the selected ADC regular channel to be converted. */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  //sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (BoardVersion == USBPD_BOARDVERSION_MB1257_B)
  {
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; 
  }
  else
  {
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  }
  
  for(ch=0;ch<USBPDM1_ADCn;ch++)
  {
    sConfig.Channel = USBPDM1_ADCs[ch].ADCCH;
    HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  }
  
  /* RXD Pins initialized as analog inputs */
  for(ch=0;ch<USBPDM1_RXDn;ch++)
  {
    sConfig.Channel = USBPDM1_RXD[ch].ADCCH;
    HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  }
  
  /* RXREF Pin initialized as analog inputs */
  sConfig.Channel = USBPDM1_RXREF.ADCCH;
  HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  
  /* Enable AWD on CC1 */ 
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  AnalogWDGConfig.WatchdogMode =    ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.ITMode =          DISABLE;  
  AnalogWDGConfig.Channel =         ADC_CHANNEL_15;
  
  /* Configure the analog watchdog */    
  AnalogWDGConfig.HighThreshold =   MV2ADC(800);
  AnalogWDGConfig.LowThreshold =    MV2ADC(400);

  HAL_ADC_AnalogWDGConfig(&usbpdm1_hadc, &AnalogWDGConfig);
}

void USBPDM1_ADCAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;
  
  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  for(ch=0;ch<USBPDM1_ADCn;ch++)
  {
    GPIO_InitStruct.Pin = USBPDM1_ADCs[ch].GPIO_Pin;
    HAL_GPIO_Init(USBPDM1_ADCs[ch].GPIOx, &GPIO_InitStruct);
  }
}

void USBPDM1_ADCAnalogGPIO_DeInit(void)
{
  uint8_t ch = 0;
  /* De-initialize GPIO pin of the selected ADC channel */
  
  for(ch=0;ch<USBPDM1_ADCn;ch++)
  {
    HAL_GPIO_DeInit(USBPDM1_ADCs[ch].GPIOx, USBPDM1_ADCs[ch].GPIO_Pin);
  }
}

void USBPDM1_ADCDMA_Init(void)
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
//  HAL_NVIC_SetPriority(ADC1_COMP_IRQn, USBPD_LOWEST_IRQ_PRIO, 0);
//  HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

void USBPDM1_ADCDMA_DeInit(void)
{
  /* De-Initialize the DMA associated to the peripheral */
  if(usbpdm1_hadc.DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(usbpdm1_hadc.DMA_Handle);
  }
}

void USBPDM1_COMPAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;
  
  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  /* RXD Pins initialized as analog inputs */
  for(ch=0;ch<USBPDM1_RXDn;ch++)
  {
    GPIO_InitStruct.Pin = USBPDM1_RXD[ch].GPIO_Pin;
    HAL_GPIO_Init(USBPDM1_RXD[ch].GPIOx, &GPIO_InitStruct);
  }
  
  /* RXREF Pin initialized as analog inputs */
  GPIO_InitStruct.Pin = USBPDM1_RXREF.GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_RXREF.GPIOx, &GPIO_InitStruct);
}

void USBPDM1_COMPAnalogGPIO_DeInit(void)
{
  uint8_t ch = 0;
  /* De-initialize GPIO pin of the selected ADC channel */
  
  for(ch = 0; ch < USBPDM1_RXDn; ch++)
  {
    HAL_GPIO_DeInit(USBPDM1_RXD[ch].GPIOx, USBPDM1_RXD[ch].GPIO_Pin);
  }
  HAL_GPIO_DeInit(USBPDM1_RXREF.GPIOx, USBPDM1_RXREF.GPIO_Pin);
}

void USBPD_HW_IF_GlobalHwInit(void)
{
#if !defined(USBPD_TCPM_MODULE_ENABLED)
  /* used for the PRL/PE timing */
  USBPD_TIM_Init();

  BoardVersion = USBPD_HW_IF_GetBoardVersion();

  /* Configure the ADCx peripheral */
  USBPDM1_ADC_Init();
  
  /* Run the ADC calibration */
  HAL_ADCEx_Calibration_Start(&usbpdm1_hadc);
  
  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&usbpdm1_hadc, ADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE);
  
  USBPDM1_CRC_Init();
  USBPDM1_DigitalGPIO_Init();
  
  /* Enabling COMP output on PA11 only for debug */
  /*
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_COMP1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */
#endif
}

HAL_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role)
{
  HAL_StatusTypeDef res = HAL_OK;

  /* Initialize default values for USBPD_HW_IF ports */
  Ports[PortNum].CCx        = CCNONE;
  Ports[PortNum].Lock       = HAL_UNLOCKED;
  Ports[PortNum].State      = HAL_USBPD_PORT_STATE_RESET;
  Ports[PortNum].ErrorCode  = 0;
  
  if (PortNum == USBPD_PORT_0)
  {
    Ports[USBPD_PORT_0].Instance   = USBPD_PORT_0;
    Ports[USBPD_PORT_0].pTxBuffPtr = (uint8_t*)TXBuffer0;
    Ports[USBPD_PORT_0].TxXferSize = DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t));
    Ports[USBPD_PORT_0].pRxBuffPtr = RXBuffer0;
    Ports[USBPD_PORT_0].pRxDataPtr = RXData0;
    Ports[USBPD_PORT_0].RxXferSize = PHY_MAX_RAW_SIZE;
  }
  
#if (USBPD_PORT_COUNT == 2)
  if (PortNum == USBPD_PORT_1)
  {
    Ports[USBPD_PORT_1].Instance   = USBPD_PORT_1;
    Ports[USBPD_PORT_1].pTxBuffPtr = (uint8_t*)TXBuffer1;
    Ports[USBPD_PORT_1].TxXferSize = DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t));
    Ports[USBPD_PORT_1].pRxBuffPtr = RXBuffer1;
    Ports[USBPD_PORT_1].pRxDataPtr = RXData1;
    Ports[USBPD_PORT_1].RxXferSize = PHY_MAX_RAW_SIZE;
  }    
#endif

  /* Reset the BIST index*/
  Ports[PortNum].BIST_index= 0;

  /* Reset the Bypass Bus Idle Flag */
  Ports[PortNum].BusIdleFlg = 0;

  /* Init of all the peripheral for the specified port */
  USBPDM1_DMA_Init(PortNum);
  USBPDM1_SPI_Init(PortNum);
  USBPDM1_TX_TIM_Init(PortNum);
  USBPDM1_COMP_Init(PortNum);
  HAL_COMP_Start(&(Ports[PortNum].hcomprx));
  USBPDM1_COUNTTIM_Init(PortNum);
  USBPDM1_RX_TIM_Init(PortNum);
  
  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(RX_TIM_IRQN(PortNum), RX_TIM_IRQ_PRIO(PortNum), 0);
  HAL_NVIC_EnableIRQ(RX_TIM_IRQN(PortNum));
  
  /* Set the power role of the port */
  USBPDM1_SetRole(PortNum,role);
  
  /* Initialize State and callbacks */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  Ports[PortNum].cbs = cbs;
  Ports[PortNum].role = role;
  
  return res;
}

void USBPDM1_CRC_Init(void)
{
  hcrc.Instance = CRC;
  
  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;      /* The default polynomial is used */
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;      /* The default init value is used */
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;   /* The input data are inverted by WORD */
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;/* The output data are Bit-reversed format */
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;     /* The input data are 32 bits lenght */
  
  HAL_CRC_Init(&hcrc);
}

uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint8_t len)
{
  uint32_t crc = 0;
  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)pBuffer, len);
  crc ^= 0xFFFFFFFF;
  return crc;
}

void USBPDM1_COMP_Init(uint8_t PortNum)
{
  COMP_HandleTypeDef* phcomprx = &(Ports[PortNum].hcomprx);
  
  phcomprx->Instance               = RX_COMP(PortNum);
  phcomprx->Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  phcomprx->Init.InvertingInput    = RX_CC1_COMPCH(PortNum);
  phcomprx->Init.Output            = RX_COMPOUT(PortNum);
  phcomprx->Init.OutputPol         = COMP_OUTPUTPOL_NONINVERTED;
  phcomprx->Init.Hysteresis        = COMP_HYSTERESIS_HIGH;
  phcomprx->Init.WindowMode        = (RX_COMP(PortNum)==COMP2) ? COMP_WINDOWMODE_ENABLE : COMP_WINDOWMODE_DISABLE;
  phcomprx->Init.TriggerMode       = COMP_TRIGGERMODE_IT_RISING_FALLING;
  phcomprx->Init.Mode              = COMP_MODE_MEDIUMSPEED;
  
  HAL_COMP_Init(phcomprx);
}

void USBPDM1_COMP_SetCC(uint8_t PortNum, CCxPin_TypeDef ccx)
{
  if (ccx == CCNONE)
  {
    /* Reinit both RXD */
  }
  else
  {
    /* in windows mode, COMP2 is slave of COMP1 master. Then need to enable also COMP1 */
    LL_COMP_Enable(COMP1);
    Ports[PortNum].hcomprx.Init.InvertingInput = (ccx == CC1) ? RX_CC1_COMPCH(PortNum) : RX_CC2_COMPCH(PortNum);
    HAL_COMP_Init(&(Ports[PortNum].hcomprx));
  }
}

void USBPDM1_DMA_Init(uint8_t PortNum)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
}

/* SPI init function */
void USBPDM1_SPI_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  SPI_HandleTypeDef*           phspi = &(Ports[PortNum].hspitx);
  
  phspi->Instance               = TX_SPI(PortNum);
  phspi->Init.Mode              = SPI_MODE_SLAVE;
  phspi->Init.Direction         = SPI_DIRECTION_2LINES;
  phspi->Init.DataSize          = SPI_DATASIZE_8BIT;
  phspi->Init.CLKPolarity       = SPI_POLARITY_LOW;
  phspi->Init.CLKPhase          = SPI_PHASE_1EDGE;
  phspi->Init.NSS               = SPI_NSS_SOFT;
  phspi->Init.FirstBit          = SPI_FIRSTBIT_LSB;
  phspi->Init.TIMode            = SPI_TIMODE_DISABLE;
  phspi->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  phspi->Init.CRCPolynomial     = 10;
  phspi->Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  phspi->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  
  HAL_SPI_Init(phspi);
}

/* TX TIM init function */
void USBPDM1_TX_TIM_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef*           phtimtx = &(Ports[PortNum].htimtx);
  TIM_OC_InitTypeDef           sConfigOC;
  uint32_t Period             = HAL_RCC_GetHCLKFreq()/BMC_TX_FREQ;

  phtimtx->Instance           = TX_TIM(PortNum);
  phtimtx->Init.Prescaler     = 0;
  phtimtx->Init.CounterMode   = TIM_COUNTERMODE_UP;
  phtimtx->Init.Period        = (Period - 1);
  phtimtx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(phtimtx);

  HAL_TIM_PWM_Init(phtimtx);

  sConfigOC.OCMode        = TIM_OCMODE_PWM1;
  sConfigOC.Pulse         = (Period/2 - 1);
  sConfigOC.OCPolarity    = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode    = TIM_OCFAST_DISABLE;
  sConfigOC.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCIdleState   = TIM_OCIDLESTATE_SET;
  
  HAL_TIM_PWM_ConfigChannel(phtimtx, &sConfigOC, TX_TIMCH(PortNum));
}

/* RX TIM init function */
void USBPDM1_RX_TIM_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef*           phtimrx = &(Ports[PortNum].htimrx);
  
  TIM_ClockConfigTypeDef   sClockSourceConfig;
  TIM_MasterConfigTypeDef   sMasterConfig;
  TIM_IC_InitTypeDef     sConfigIC;
  
  phtimrx->Instance           = RX_TIM(PortNum);
  phtimrx->Init.Prescaler     = ( HAL_RCC_GetHCLKFreq() / ( 20 * BMC_TX_FREQ) ) - 1 ; // 3;
  phtimrx->Init.CounterMode   = TIM_COUNTERMODE_UP;
  phtimrx->Init.Period        = 255;
  phtimrx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  HAL_TIM_Base_Init(phtimrx);
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(phtimrx, &sClockSourceConfig);
  
  HAL_TIM_IC_Init(phtimrx);
  
  sMasterConfig.MasterOutputTrigger =     TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode =         TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(phtimrx, &sMasterConfig);
  
  sConfigIC.ICPolarity =    TIM_TRIGGERPOLARITY_RISING; //TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection =   TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler =   TIM_ICPSC_DIV1;
  sConfigIC.ICFilter =       0;
  
  HAL_TIM_IC_ConfigChannel(phtimrx, &sConfigIC, RX_TIMCH(PortNum));
}

/* COUNTTIM init function */
void USBPDM1_COUNTTIM_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef* htimcountrx = &(Ports[PortNum].htimcountrx);
  
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  htimcountrx->Instance =                       RX_COUNTTIM(PortNum);
  htimcountrx->Init.Prescaler =                 ( HAL_RCC_GetHCLKFreq() / 1000000 ) - 1; // 1us Resolution
  htimcountrx->Init.CounterMode =               TIM_COUNTERMODE_UP;
  htimcountrx->Init.Period =                    DMA_TIME_ELAPSED;
  htimcountrx->Init.ClockDivision =             TIM_CLOCKDIVISION_DIV1;
  htimcountrx->Init.RepetitionCounter =         0;
  HAL_TIM_Base_Init(htimcountrx);
  
  HAL_TIM_OC_Init(htimcountrx);
  
  sBreakDeadTimeConfig.OffStateRunMode =        TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode =       TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel =              TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime =               0;
  sBreakDeadTimeConfig.BreakState =             TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity =          TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput =        TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(htimcountrx, &sBreakDeadTimeConfig);
  
  sConfigOC.OCMode =                            TIM_OCMODE_TIMING;
  sConfigOC.Pulse =                             DMA_TIME_COUNT_COMPARE;
  sConfigOC.OCPolarity =                        TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity =                       TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode =                        TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState =                       TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState =                      TIM_OCNIDLESTATE_RESET;
  HAL_TIM_OC_ConfigChannel(htimcountrx, &sConfigOC, RX_COUNTTIMCH(PortNum));
  
  __HAL_TIM_CLEAR_IT(htimcountrx, TIM_IT_UPDATE);
}

void USBPDM1_TX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);
  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].hspitx.hdmatx = hdma_tx_spi;
  
  hdma_tx_spi->Instance =                   TX_DMACH(PortNum);
  hdma_tx_spi->Init.Direction =             DMA_MEMORY_TO_PERIPH;
  hdma_tx_spi->Init.PeriphInc =             DMA_PINC_DISABLE;
  hdma_tx_spi->Init.MemInc =                DMA_MINC_ENABLE;
  hdma_tx_spi->Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
  hdma_tx_spi->Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
  hdma_tx_spi->Init.Mode =                  DMA_NORMAL;
  hdma_tx_spi->Init.Priority =              DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(hdma_tx_spi);
  
  __HAL_LINKDMA((&Ports[PortNum].hspitx),hdmatx,(*hdma_tx_spi));
  
  /* DMA interrupt init */
  HAL_NVIC_SetPriority(TX_DMACHIRQ(PortNum), TX_DMACHIRQ_PRIO(PortNum), 0);
}

void USBPDM1_RX_EnableInterrupt(uint8_t PortNum)
{
  /* Set the port state to waiting */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  __HAL_TIM_ENABLE(&(Ports[PortNum].htimrx));
  /* Add enable the DMA */
  /* Enable the Rx interrupt if cbs are initialized; this mean that this is a PD capable port */
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  HAL_TIM_IC_Start_IT (&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
}

void USBPDM1_RX_DisableInterrupt(uint8_t PortNum)
{
  /* The port is ready to transmit */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  
  __HAL_TIM_DISABLE(&(Ports[PortNum].htimrx));
  
  SINGLE_TIM_IC_Stop_IT(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum), RX_TIMCH_TIMIT(PortNum));
  /* Stop DMA transfers */
  __HAL_TIM_DISABLE_DMA(&(Ports[PortNum].htimrx), RX_TIM_DMA_CC(PortNum));
  /* Change the htim state */
  (&Ports[PortNum].htimrx)->State = HAL_TIM_STATE_READY;
  /* stop the TIM counter for the selected port */
  SINGLE_TIM_OC_Stop_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum), RX_COUNTTIMCH_TIMIT(PortNum));
  /* Disable the TIM Update interrupt */
  __HAL_TIM_DISABLE_IT(&(Ports[PortNum].htimcountrx), TIM_IT_UPDATE);
  /* Reset the counter */
  Ports[PortNum].htimcountrx.Instance->CNT = 0;
  
  /* Add disable the DMA */
  __HAL_DMA_DISABLE(&(Ports[PortNum].hdmarx));
}

void USBPDM1_RX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_rx_tim = &Ports[PortNum].hdmarx;
  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].htimrx.hdma[TIM_DMA_ID_CC1] = hdma_rx_tim;
  
  /* Rx DMA init*/
  hdma_rx_tim->Instance                 = RX_DMACH(PortNum);
  hdma_rx_tim->Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx_tim->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx_tim->Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx_tim->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_rx_tim->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx_tim->Init.Mode                = DMA_NORMAL;
  hdma_rx_tim->Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(hdma_rx_tim);
  
  /* Several peripheral DMA handle pointers point to the same DMA handle.
  Be aware that there is only one channel to perform all the requested DMAs. */
  __HAL_LINKDMA(&(Ports[PortNum].htimrx),hdma[TIM_DMA_ID_CC1],(*hdma_rx_tim));
}

void USBPDM1_RX_DMA_Deinit(uint8_t PortNum)
{
  /* Peripheral DMA DeInit*/
  HAL_DMA_DeInit(Ports[PortNum].htimrx.hdma[TIM_DMA_ID_CC1]);
}

extern uint8_t PE_GetSwapOngoing(uint8_t PortNum);

USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize)
{
  /* Check if the port is yet receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
    return USBPD_BUSY;

  USBPD_StatusTypeDef ret = USBPD_ERROR;
    
  uint16_t size=0;  
  uint16_t *pTxBuffer = (uint16_t *)Ports[PortNum].pTxBuffPtr;
  uint16_t *pTxDataBuffer = pTxBuffer+(TX_PREAMBLE_SIZE/2);     /* pointer of the first data, after the preamble */
  uint16_t sTxDataBufferBitsize = 0;                            /* size of data (SOP ... EOP) */
  uint16_t sStartupValue = 0;
  uint16_t sMaxTXDataBufferSize = TX_BUFFER_LEN*4 - TX_PREAMBLE_SIZE; /* bytes */
  uint8_t nLastBit = 0x00;
  uint16_t nTotalBitsize = 0x00;

  memset((uint8_t *)pTxBuffer, 0x00, TX_BUFFER_SIZE);
  memset((uint8_t *)pTxBuffer, TX_PREAMBLE_BMC_CODED, TX_PREAMBLE_SIZE); /* preamble length */
  
  sStartupValue = *(uint16_t *)(pTxDataBuffer-1); /* get the previous value (it is preamble) */
  ret = BMC_MakeCoding(pBuffer, Bitsize, pTxDataBuffer, &sTxDataBufferBitsize, sMaxTXDataBufferSize, sStartupValue, &nLastBit);
  
  /* Add last edge */
  nTotalBitsize = TX_PREAMBLE_SIZE*8 + sTxDataBufferBitsize;
  if (nLastBit == 0x00) 
  {
    uint16_t *pLastItem = (uint16_t *)&(pTxBuffer[nTotalBitsize / 16]);
    uint16_t edgeBit = nTotalBitsize % 16;
    *pLastItem |= 3<<edgeBit;
    if (edgeBit == 15)
    {
      *(pLastItem + 1) = 0x0001;
    }
    nTotalBitsize += 3;
  }
  else
  {
    nTotalBitsize += 1; /* adding a zero for the tHoldLowBMC */
  }

  if (ret != USBPD_OK)
    return ret;

  /* Check if the port is yet receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
  {
    return USBPD_BUSY;
  }

  /* evaluate the number of data byte to be transmitted */
  size = DIV_ROUND_UP(nTotalBitsize, 8);

  /* Disable the Rx process */
  USBPDM1_RX_DisableInterrupt(PortNum);
    
  /* Set the state to busy*/
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_TX;


  /* Check if the port is yet receiving */
  ret = USBPD_OK; //HW_IF_CheckBusIdle(PortNum);
  
  if (PE_GetSwapOngoing(PortNum) == 0)
  {
    /* Check if the port is yet receiving */
    ret = HW_IF_CheckBusIdle(PortNum);
  }

  /* if the bus is idle send data */
  if (ret == USBPD_OK)
  {
    /* Enables the TX TC Interrupt */
    HAL_NVIC_EnableIRQ(TX_DMACHIRQ(PortNum));

    /* Set the correct GPIOs */
    USBPDM1_DeInitRXD(PortNum, Ports[PortNum].CCx);    
    
    /* Set the pin to be used for transmission by SPI */
    USBPDM1_SPI_Set_TX_CC(PortNum, Ports[PortNum].CCx);

    if ( (Ports[PortNum].role == USBPD_PORTPOWERROLE_SRC) || (Ports[PortNum].role == USBPD_PORTPOWERROLE_DRP_SRC) )
    {
      USBPDM1_DeAssertRp(PortNum);
    }
    
    /* Start the timer clocking the SPI */  
    HAL_TIM_PWM_Start(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));

    /* Start transmission */
    SINGLE_SPI_Transmit_DMA(&(Ports[PortNum].hspitx), Ports[PortNum].pTxBuffPtr, size);

    /* PC01 A8 reset : start of the transmission */
    //GPIOA->BRR = GPIO_PIN_8;
  }  
  else 
  {
    /* Enable RX Interrupt */
    USBPDM1_RX_EnableInterrupt(PortNum);
  }
  return ret;
}

/**
  * @brief  Bypass checking if bus IDLE.
  * @param  PortNum Current port number.
  * @param  State   BUS state to check
  * @retval none 
  */
void USBPD_HW_IF_CheckBusIdleState(uint8_t PortNum, FunctionalState State)
{
  Ports[PortNum].BusIdleFlg = DISABLE == State ? 1 : 0;
}

/**
  * @brief  Check the bus status.
  * @param  PortNum Specifies the port to check.
  * @retval USBPD_OK => bus idle, USBPD_BUSY => bus busy, error and timeout is not allowed
  */

  uint8_t ADC_AWDEvent = 0;
  uint32_t ADC_AWDEventCount = 0;
  uint8_t CCPortIndex = 0;
  uint8_t CCPortADCCh = 0;
  uint32_t CCAnalogValue = 0;
  uint16_t ICDiff = 0;
USBPD_StatusTypeDef HW_IF_CheckBusIdle(uint8_t PortNum)
{
  int i = 0;

  /*
   | AWD | IC | ADC level | Condition | Note
  -+-----+----+-----------+-------------------------------------------------
   |  0  |  0 |     -     |    idle   | Strange case, idle
   |  0  |  1 |     -     |    idle   | Getting events inside the noise area
   |  1  |  0 |    >0V    |    idle   | Out of high level
   |  1  |  0 |    ~0V    |    busy   | Out of low level (formally 0V)
   |  1  |  1 |     -     |    busy   | Getting events out of the noise area
  
  ADC Mapping
  Single Port
  P0 CC1 = Channel 0
  P0 CC2 = Channel 3
  
  Dual Port
  P0 CC1 = Channel 0
  P0 CC2 = Channel 5
  P1 CC1 = Channel 2
  P1 CC2 = Channel 4
  */
  
  /* Bypass the check bus idle */  
  if(Ports[PortNum].BusIdleFlg == 1)
  {
    return USBPD_OK;
  }
  
  USBPD_StatusTypeDef ret = USBPD_OK;
  
  /* PC02 B0 set : time added for check bus idle */
  //GPIOB->BSRR = GPIO_PIN_0;
  
  /****** Check edges ******/
  /* configure the input capture timer on both edge to get possible transitions in CCx line */
  Ports[PortNum].htimrx.Instance->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP; //BOTHEDGE

  /* start the counting of the possible transitions in CCx line */
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  __HAL_TIM_ENABLE_DMA(&(Ports[PortNum].htimrx), RX_TIM_DMA_CC(PortNum));
  HAL_TIM_IC_Start(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));

  /****** Check edges end ******/

  /****** ADC/AWD configuration ******/
  /* noise definition: 
  PD Communications Engine USB PD Compliance MOI
  version 1.0
  June 9, 2016
  3.4.1 TDA.1.1.2.1.1 
  It shall be a square wave at a frequency of 600kHz
  It shall have an amplitude of 250mV p/p, biased around 0.55V
  */
  CCPortIndex = CC_INDEX(PortNum,Ports[PortNum].CCx);
  CCPortADCCh = CC_ADC_CHANNEL(PortNum,Ports[PortNum].CCx);

  /* enable analog watchdog in CCx line to recognize if noise or transmission */    
  usbpdm1_hadc.Instance->ISR |= ADC_FLAG_AWD; /* reset the AWD events */
  usbpdm1_hadc.Instance->CFGR1 &= ~(ADC_CFGR1_AWD1CH_Msk); /* set 0 the channels in the register */
  usbpdm1_hadc.Instance->CFGR1 |= ADC_ANALOGWATCHDOG_SINGLE_REG | (CCPortADCCh<<ADC_CFGR1_AWD1CH_Pos); /* set the AWD single channel and set the channel */
//  usbpdm1_hadc.Instance->SMPR = ADC_SAMPLETIME_7CYCLES_5 & 0x7;

  /* wait 12-20us */
  /* PC03 B0 set/reset : time to evaluate edge */
//  GPIOB->BSRR = GPIO_PIN_0;
  for(i = 0; i<120; i++) __NOP();
//  GPIOB->BRR = GPIO_PIN_0;

  /* Get the bus value */
  CCAnalogValue = ADCxConvertedValues[CCPortIndex];
  
  /* Get AWD Event */
  ADC_AWDEvent = usbpdm1_hadc.Instance->ISR & ADC_FLAG_AWD;
  
  /* After 12 - 20 us Stop the AWD */
  usbpdm1_hadc.Instance->CFGR1 &= ~ADC_ANALOGWATCHDOG_SINGLE_REG;
  
  /* Evaluate number RX Events */
  ICDiff = PHY_MAX_RAW_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR;
  
  if (ADC_AWDEvent == 0) 
  {
    ADC_AWDEventCount++;
  }

  /* Check Bus Condition */
  if ((ADC_AWDEvent > 0 ) && (ICDiff >= BUSCHECK_IC_NUMBER || CCAnalogValue <= BUSCHECK_THRESH_LOW))
  {
    ret = USBPD_BUSY;
  }
  
  /* set the interrupt of the timer to get rising events */
  Ports[PortNum].htimrx.Instance->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //RISING
  
  /* PC02 B0 reset : time added for check bus idle */
//  GPIOB->BRR = GPIO_PIN_0;
  
  return ret;
}

HAL_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  
  /* BIST Carrier mode flag set */
  Ports[PortNum].State=HAL_USBPD_PORT_STATE_BIST;
  
  /* Fill the buffer with the pattern to be sent */
  memset(Ports[PortNum].pTxBuffPtr, 0xB4, TX_BUFFER_LEN*4);
  
  /* start a circular DMA transfer */
  USBPDM1_Set_DMA_Circular_Mode(PortNum);
  
  /* Disable the Rx process */
  USBPDM1_RX_DisableInterrupt(PortNum);
  /* Enables the TX TC Interrupt */
  HAL_NVIC_EnableIRQ(TX_DMACHIRQ(PortNum));
  /* Set the state to busy*/
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BIST;
  /* Start the timer clocking the SPI */  
  HAL_TIM_PWM_Start(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));
  /* Set the pin to be used for transmission by SPI */
  USBPDM1_SPI_Set_TX_CC(PortNum, Ports[PortNum].CCx);
  /* Set the correct GPIOs */
  USBPDM1_DeInitRXD(PortNum, Ports[PortNum].CCx);
  
  /* Start transmission */
  HAL_SPI_Transmit_DMA(&(Ports[PortNum].hspitx), (uint8_t*)(Ports[PortNum].pTxBuffPtr), TX_BUFFER_LEN*4);
    
  ret = HAL_OK;
  return ret;
}

void USBPDM1_Set_DMA_Normal_Mode(uint8_t PortNum)
{
  LL_DMA_SetMode(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum), LL_DMA_MODE_NORMAL);
}

void USBPDM1_Set_DMA_Circular_Mode(uint8_t PortNum)
{
  LL_DMA_SetMode(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum), LL_DMA_MODE_CIRCULAR);
  
  /* DMA interrupt init */
  HAL_NVIC_SetPriority(TX_DMACHIRQ(PortNum), TX_DMACHIRQ_PRIO(PortNum), 0);
}

void USBPDM1_TX_Done(uint8_t PortNum)
{
  /* Wait for FIFO empty */
  while (LL_SPI_GetTxFIFOLevel(Ports[PortNum].hspitx.Instance) != LL_SPI_TX_FIFO_EMPTY);
  /* Wait for BUSY flag */
  while (LL_SPI_IsActiveFlag_BSY(Ports[PortNum].hspitx.Instance));
  /* Here the SPI has completed the transmission*/
  /* Stop SPI */
  LL_SPI_Disable(Ports[PortNum].hspitx.Instance); /*->CR1 &= 0xFFBF;*/
  
  /* Configure RXD as RX pin */
  USBPDM1_ReInitRXD(PortNum, Ports[PortNum].CCx);
  /* Put SPI MISO pin in high impedence */
  USBPDM1_SPI_Reset_TX_CC(PortNum, Ports[PortNum].CCx);

  if ( (Ports[PortNum].role == USBPD_PORTPOWERROLE_SRC) || (Ports[PortNum].role == USBPD_PORTPOWERROLE_DRP_SRC) )
  {
    if ((PE_GetSwapOngoing(PortNum) & 0x40) == 0)
    {
      USBPDM1_AssertRp(PortNum);
    }
  }

  /* Clean DMA flags */
  HAL_DMA_IRQHandler(&Ports[PortNum].hdmatx);
  /* Disable TX interrupt */
  HAL_NVIC_DisableIRQ(TX_DMACHIRQ(PortNum));

  /* Stop the timer clocking the SPI */
  HAL_TIM_PWM_Stop(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));

  /* Check if BIST TX Done */
  if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
  {
    Ports[PortNum].State=HAL_USBPD_PORT_STATE_RESET;
    /* Evaluate callback*/
    if ((Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted != NULL) )/*&& (bitsize>0) )*/
    {
      Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted(PortNum,USBPD_BIST_CARRIER_MODE2);
    }
  }
  
  /* Enable RX Interrupt */
  USBPDM1_RX_EnableInterrupt(PortNum);
  
  if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
  {
      Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum);
  }
}

__STATIC_INLINE void USBPDM1_DeInitRXD(uint8_t PortNum,CCxPin_TypeDef cc)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Configure the GPIO pin */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  /* Put the RXD pin to GND to allow transmission */
  switch (cc)
  {
  case CC1:
    GPIO_InitStruct.Pin = RX_CC1_PIN(PortNum);
    SINGLE_GPIO_Init(RX_CC1_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC1_PIN_POSITION(PortNum));
    (RX_CC1_GPIOPORT(PortNum))->BRR = (uint32_t)RX_CC1_PIN(PortNum);
    break;
  case CC2:
    GPIO_InitStruct.Pin = RX_CC2_PIN(PortNum);
    SINGLE_GPIO_Init(RX_CC2_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC2_PIN_POSITION(PortNum));
    (RX_CC2_GPIOPORT(PortNum))->BRR = (uint32_t)RX_CC2_PIN(PortNum);
    break;
  default:
    break;
  }
}

__STATIC_INLINE void USBPDM1_ReInitRXD(uint8_t PortNum,CCxPin_TypeDef cc)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  /* Reconfigure the RXD pin for reception */
  switch (cc)
  {
  case CC1:
    GPIO_InitStruct.Pin = RX_CC1_PIN(PortNum);
    SINGLE_GPIO_DeInit(RX_CC1_GPIOPORT(PortNum),RX_CC1_PIN(PortNum),RX_CC1_PIN_POSITION(PortNum));
    SINGLE_GPIO_Init(RX_CC1_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC1_PIN_POSITION(PortNum));
    break;
  case CC2:
    GPIO_InitStruct.Pin = RX_CC2_PIN(PortNum);
    SINGLE_GPIO_DeInit(RX_CC2_GPIOPORT(PortNum),RX_CC2_PIN(PortNum),RX_CC2_PIN_POSITION(PortNum));
    SINGLE_GPIO_Init(RX_CC2_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC2_PIN_POSITION(PortNum));
    break;
  default:
    break;
  }    
}

void RX_Init_Hvar(uint8_t PortNum)
{
  /* Decoding variables */
  FiveBitCodingVar_TypeDef* hvar = &(Ports[PortNum].decfields);
  
  /* Set the state of the port */
  Ports[PortNum].State =     HAL_USBPD_PORT_STATE_BUSY_RX;
  /* Init the decoding variables */

  hvar->preamble =         0;
  hvar->k =                0;
  hvar->j =                0;
  hvar->curr_indx =        0;
  hvar->prev_bit =         0;
  hvar->exed_flag =        0;
  hvar->temp_data =        0;
  
  /* reset the Rx Raw data buffer (in diff edge BMC coded) */
  memset(Ports[PortNum].pRxBuffPtr, 0, PHY_MAX_RAW_SIZE);

  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset != NULL) 
  {
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset(PortNum);
  }
}

__STATIC_INLINE void HW_IF_DecodingTask(uint8_t PortNum, uint8_t Phase)
{
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate == NULL) 
  {
    return;
  }

  uint16_t last_index=0; 
  uint8_t  curr_bit=0;
  uint8_t  diff_noise;
  
  /* Get the peripheral handler */
  FiveBitCodingVar_TypeDef* hvar = &(Ports[PortNum].decfields);
  /* RX2 is the pointer to raw data buffer */
  uint8_t* RX2 = Ports[PortNum].pRxBuffPtr; //DMA data -- input buffer
  /*RX1 is the final destination buffer with data without Preamble */
  
  /* calculate available sample in the dma */ 
  last_index = (PHY_MAX_RAW_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR) + 1;

  if (last_index > 1)
  {
    /* check preamble */
    if (hvar->preamble == 0)
    {
      while (1)
      {
        /* decoded all available data */
        if (hvar->curr_indx >= last_index)
          break;
        
        /* the cpu resource is finished */
        if (Phase == 0 && Ports[PortNum].htimcountrx.Instance->CNT >= DMA_TIME_TASK) 
          break;

        /* calculate the difference with a safe threshold */
        diff_noise=(RX2[hvar->curr_indx]-RX2[hvar->curr_indx-1]+PARAM_RX_OFFSET);

        /* increment the index */
        hvar->curr_indx++;

        /* to avoid glitch in presence of noise */
        if (diff_noise <= (3+PARAM_RX_OFFSET))
        {
          continue;
        }
        
        /* calculate the current bit diff/30 and not */
        curr_bit = !( (diff_noise>>5) & 0x01 );

        /* increment the index if detect a 1 */
        hvar->curr_indx+=curr_bit;
        
        /* verify the end of the preamble checking for two consecutive bit equal. 
        * Shift first five ones */
        if ((curr_bit==hvar->prev_bit) && (hvar->curr_indx>10))
        {  
          /* set the preamble flag to 1 */
          hvar->preamble = 1;
          hvar->temp_data = 0;
          if(curr_bit==0)
          {
            /* set 0 to position  0 to recover the first 0 */
            hvar->temp_data |= curr_bit;
            hvar->j=1;
          }
          /* set next bit */
          hvar->temp_data |= (curr_bit<<hvar->j);
          hvar->j++;
          break; /* the preamble is completed */
        }

        /* store bit value for end of preamble evaluation */
        hvar->prev_bit = curr_bit;
      }
    }

    /* if a preamble is detected starting the decoding */
    if (hvar->preamble == 1)
    {
      while (1)
      {
        /* decoded all available data */
        if (hvar->curr_indx >= last_index)
          break;
        
        /* the cpu resource is finished */
        if (Phase == 0 && Ports[PortNum].htimcountrx.Instance->CNT >= DMA_TIME_TASK && hvar->j>5) 
          break;
        
        /* calculate the difference with a safe threshold */
        diff_noise=(RX2[hvar->curr_indx]-RX2[hvar->curr_indx-1]+PARAM_RX_OFFSET);
        
        /* increment the index */
        hvar->curr_indx++;
        
        /* to avoid glitch in presence of noise */
        if (diff_noise <= (3+PARAM_RX_OFFSET))
        {
          continue;
        }

        /* calculate the current bit diff/30 and not */
        curr_bit = !( (diff_noise>>5) & 0x01 );
        
        /* increment the index if detect a 1 */
        hvar->curr_indx+=curr_bit;
        
        /* write next bits */
        hvar->temp_data |= (curr_bit << hvar->j);
        hvar->j++;
        
        if (hvar->j==10)
        {
          Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, hvar->temp_data);
          hvar->j=0;
          hvar->k++; /* only used to check the overflow */
          hvar->temp_data = 0;

          /* re-calculate the last index */
          //last_index = (PHY_MAX_RAW_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR) + 1;
        }
        if (hvar->j==5 && hvar->temp_data == 0x0D) //EOP
        {
          Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, hvar->temp_data);
          hvar->exed_flag = 2;
        }
      }
    }
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t PortNum;
  uint32_t bitsize = 0;
  uint32_t tim_count = 0;
  uint32_t dma_count = 0;
  uint32_t dma_count2 = 0;
  
#if (USBPD_PORT_COUNT==1)
  PortNum = 0;
#elif (USBPD_PORT_COUNT==2)           
  PortNum = ( (htim->Instance == RX_COUNTTIM(0)) ? 0 : 1);
#endif /* RX TIMER IDENTIFIED */

  /* The message has been transmitted if non change on DMA->CNDTR*/
  Ports[PortNum].decfields.DMA_count = Ports[PortNum].hdmarx.Instance->CNDTR;
  
  /* perform a decoding task in any case */
  //GPIOA->BSRR = GPIO_PIN_8;
  HW_IF_DecodingTask(PortNum, 0);
  //GPIOA->BRR = GPIO_PIN_8;
  
  dma_count = 0;
  if (Ports[PortNum].decfields.exed_flag == 0)
  {
    /* get the tim count with a safe check */
    tim_count = MIN(Ports[PortNum].htimcountrx.Instance->CNT, DMA_TIME_THRESHOLD1);
    
    /* waiting 5us if there is enough time */
    while (Ports[PortNum].htimcountrx.Instance->CNT < (tim_count + DMA_TIME_DURATION));

    /* wait few us and so check again the dma count */
    tim_count = MIN(Ports[PortNum].htimcountrx.Instance->CNT, DMA_TIME_THRESHOLD2);
    dma_count = Ports[PortNum].hdmarx.Instance->CNDTR;
    
    //GPIOA->BRR = GPIO_PIN_8;
    while (Ports[PortNum].htimcountrx.Instance->CNT < tim_count + DMA_TIME_DURATION);
    //GPIOA->BSRR = GPIO_PIN_8;
    dma_count2 = Ports[PortNum].hdmarx.Instance->CNDTR;
  }

  /* check if the dma_count doesn't change */
  if (( (dma_count2 == dma_count) || (Ports[PortNum].decfields.exed_flag > 0) ))
  {
    /* PC04 A8 reset : Rx start */
    GPIOA->BRR = GPIO_PIN_8;

    /* PC01 A8 set : start of the transmission */
    //GPIOA->BSRR = GPIO_PIN_8;
    Ports[PortNum].decfields.DMA_count = dma_count;
    /* Stop the TIM DMA transfers */
    HAL_TIM_IC_Stop_DMA(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
    
    /* DMA Abort to execute the DMA Deinit */
    HAL_DMA_Abort(&Ports[PortNum].hdmarx);
    
    /* stop the TIM counter for the selected port */
    Ports[PortNum].htimcountrx.Instance->CNT = 0;
    
    /* stop the TIM counter for the selected port */    
    SINGLE_TIM_OC_Stop_IT(&(Ports[PortNum].htimcountrx), RX_COUNTTIMCH(PortNum), RX_COUNTTIMCH_TIMIT(PortNum));
    /* Disable the TIM Update interrupt */
    HAL_TIM_Base_Stop_IT(&(Ports[PortNum].htimcountrx));
    
    /* start Data reading and decoding in the final phase (1)  */
    if (Ports[PortNum].decfields.exed_flag == 0)
    {
      HW_IF_DecodingTask(PortNum, 1);
    }
    
    /* Configure RX Timer for first edge interrupt */
    Ports[PortNum].htimrx.Instance->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //RISING
    
    __HAL_TIM_CLEAR_IT(&(Ports[PortNum].htimrx), RX_TIMCH_TIMIT(PortNum)); //AR to avoid 2 interrupt RX timer callback (with a reset of the decfields vars
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;

    if (Ports[PortNum].decfields.exed_flag != 1)
    {
      /* Get the number of bit received*/
      bitsize = Ports[PortNum].decfields.k * 32 + Ports[PortNum].decfields.j;
      
      /* Evaluate callback*/
      if (( Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed != NULL) && (bitsize>0) )
      {
        Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed(PortNum);
      }
    }
    
    /* REanables RX Interrupt */
    USBPDM1_RX_EnableInterrupt(PortNum);
  }
  else
  {    
    /* start Data reading and decoding in the intermidiate phase (0) */
  }  
}

void USBPDM1_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  /* Set the correct pin on the comparator*/
  Ports[PortNum].CCx = cc;
  USBPDM1_COMP_SetCC(PortNum,cc);
}

__STATIC_INLINE void USBPDM1_SetRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role)
{
  if ((role == USBPD_PORTPOWERROLE_SRC) || (role == USBPD_PORTPOWERROLE_DRP_SRC))
  {
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
    /* SRC Case */
    if (PortNum == 0)
    {
      /* Set Rp */
      USBPDM1_AssertRp(PortNum);
      /* UnSet Rd */
      USBPDM1_GPIO_Off(HRD_P1);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P1);
      USBPDM1_GPIO_On(ENCC1_P1);
    }
#else
    /* SRC Case */
    if (PortNum == 0)
    {
      /* Set Rp */
      USBPDM1_AssertRp(PortNum);
      /* UnSet Rd */
      USBPDM1_GPIO_Off(HRD_P0);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P0);
      USBPDM1_GPIO_On(ENCC1_P0);
    }
#endif
#if (USBPD_PORT_COUNT==2)
    else if (PortNum == 1)
    {
      /* Set Rp */
      USBPDM1_AssertRp(PortNum);
      /* UnSet Rd */
      USBPDM1_GPIO_Off(HRD_P1);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P1);
      USBPDM1_GPIO_On(ENCC1_P1);
    }
#endif
    DRP_SET_PROVIDER(PortNum);
  }
  else
  {
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
    /* SNK Case*/
    if (PortNum == 0)
    {
      /* Set Rd */
      USBPDM1_GPIO_On(HRD_P1);
      /* Unset Rp */
      USBPDM1_DeAssertRp(PortNum);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P1);
      USBPDM1_GPIO_On(ENCC1_P1);
    }
#else
    /* SNK Case*/
    if (PortNum==0)
    {
      /* Set Rd */
      USBPDM1_GPIO_On(HRD_P0);
      /* Unset Rp */
      USBPDM1_DeAssertRp(PortNum);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P0);
      USBPDM1_GPIO_On(ENCC1_P0);
    }
#endif
#if (USBPD_PORT_COUNT==2)
    else if (PortNum==1)
    {
      /* Set Rd */
      USBPDM1_GPIO_On(HRD_P1);
      /* Unset Rp */
      USBPDM1_DeAssertRp(PortNum);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P1);
      USBPDM1_GPIO_On(ENCC1_P1);
    }
#endif
    /*
    Removed to allow hw to se te role
    DRP_SET_CONSUMER(PortNum);
    */
  }
}

void USBPDM1_AssertRp(uint8_t PortNum)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Configure the GPIO pin */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if (USBPD_PORT_COUNT==2)
  if (PortNum==0)
  {
#endif
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx,USBPDM1_GPIOs[HRP_P1].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P1].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
    (USBPDM1_GPIOs[HRP_P1].GPIOx)->BSRR = (uint32_t)(USBPDM1_GPIOs[HRP_P1].GPIO_Pin);
#else
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P0].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P0].GPIOx,USBPDM1_GPIOs[HRP_P0].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P0].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
    (USBPDM1_GPIOs[HRP_P0].GPIOx)->BSRR = (uint32_t)(USBPDM1_GPIOs[HRP_P0].GPIO_Pin);
#endif
#if (USBPD_PORT_COUNT==2)
  }
  else if (PortNum==1)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx,USBPDM1_GPIOs[HRP_P1].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P1].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
    (USBPDM1_GPIOs[HRP_P1].GPIOx)->BSRR = (uint32_t)(USBPDM1_GPIOs[HRP_P1].GPIO_Pin);
  }
#endif
}

void USBPDM1_DeAssertRp(uint8_t PortNum)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if (USBPD_PORT_COUNT==2)
  if (PortNum==0)
  {
#endif
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx,USBPDM1_GPIOs[HRP_P1].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P1].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
#else
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P0].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P0].GPIOx,USBPDM1_GPIOs[HRP_P0].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P0].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
#endif
#if (USBPD_PORT_COUNT==2)
  }
  if (PortNum==1)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx,USBPDM1_GPIOs[HRP_P1].GPIO_Pin,HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P1].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
  }
#endif
}

void USBPDM1_AssertRd(uint8_t PortNum)
{
#if (USBPD_PORT_COUNT==2)
  if (PortNum==0)
  {
#endif
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
      USBPDM1_GPIO_On(HRD_P1);
#else
      USBPDM1_GPIO_On(HRD_P0);
#endif
#if (USBPD_PORT_COUNT==2)
    }
    else if (PortNum==1)
    {
      USBPDM1_GPIO_On(HRD_P1);
    } 
#endif
}


void USBPDM1_DeAssertRd(uint8_t PortNum)
{
#if (USBPD_PORT_COUNT==2)
  if (PortNum==0)
  {
#endif
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
      USBPDM1_GPIO_Off(HRD_P1);
#else
      USBPDM1_GPIO_Off(HRD_P0);
#endif
#if (USBPD_PORT_COUNT==2)
    }
    else if (PortNum==1)
    {
      USBPDM1_GPIO_Off(HRD_P1);
    } 
#endif
}

void HW_SignalDetachment(uint8_t PortNum, CCxPin_TypeDef cc)
{
  USBPDM1_RX_DisableInterrupt(PortNum);
  USBPDM1_Set_CC(PortNum, CCNONE);
  USBPDM1_ReInitRXD(PortNum,cc);
}

/* -------------------------------------------------------------------------- */
/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
* @brief  De-initialize the GPIOx peripheral registers to their default reset values.
* @param  GPIOx: where x can be (A..F) to select the GPIO peripheral for STM32F0 family
* @param  GPIO_Pin: specifies the port bit to be written.
*         This parameter can be one of GPIO_PIN_x where x can be (0..15).
* @retval None
*/
__STATIC_INLINE void SINGLE_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin, uint32_t position )
{
  uint32_t tmp = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  /*------------------------- GPIO Mode Configuration --------------------*/
  /* Configure IO Direction in Input Floting Mode */
  CLEAR_BIT(GPIOx->MODER, GPIO_MODER_MODER0 << (position * 2)); 
  
  /* Configure the default Alternate Function in current IO */ 
  CLEAR_BIT(GPIOx->AFR[position >> 3], (uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;
  
  /* Configure the default value for IO Speed */
  CLEAR_BIT(GPIOx->OSPEEDR, GPIO_OSPEEDER_OSPEEDR0 << (position * 2));
  
  /* Configure the default value IO Output Type */
  CLEAR_BIT(GPIOx->OTYPER, GPIO_OTYPER_OT_0 << position) ;
  
  /* Deactivate the Pull-up oand Pull-down resistor for the current IO */
  CLEAR_BIT(GPIOx->PUPDR, GPIO_PUPDR_PUPDR0 << (position * 2));
  
  /*------------------------- EXTI Mode Configuration --------------------*/
  /* Clear the External Interrupt or Event for the current IO */
  
  tmp = SYSCFG->EXTICR[position >> 2];
  tmp &= (((uint32_t)0x0F) << (4 * (position & 0x03)));
  if(tmp == (GPIO_GET_INDEX(GPIOx) << (4 * (position & 0x03))))
  {
    tmp = ((uint32_t)0x0F) << (4 * (position & 0x03));
    CLEAR_BIT(SYSCFG->EXTICR[position >> 2], tmp);
    /* Clear EXTI line configuration */
    CLEAR_BIT(EXTI->IMR, (uint32_t)GPIO_Pin);
    CLEAR_BIT(EXTI->EMR, (uint32_t)GPIO_Pin);
    /* Clear Rising Falling edge configuration */
    CLEAR_BIT(EXTI->RTSR, (uint32_t)GPIO_Pin);
    CLEAR_BIT(EXTI->FTSR, (uint32_t)GPIO_Pin);
  }
}

/**
* @brief  Initialize the GPIOx peripheral according to the specified parameters in the GPIO_Init.
* @param  GPIOx: where x can be (A..F) to select the GPIO peripheral for STM32F0 family
* @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
*         the configuration information for the specified GPIO peripheral.
* @retval None
*/
__STATIC_INLINE void SINGLE_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init, uint32_t position)
{ 
  uint32_t temp = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));
  assert_param(IS_GPIO_PULL(GPIO_Init->Pull)); 
  
  /*--------------------- GPIO Mode Configuration ------------------------*/
  /* In case of Alternate function mode selection */
  if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD)) 
  {
    /* Check the Alternate function parameters */
    assert_param(IS_GPIO_AF_INSTANCE(GPIOx));
    assert_param(IS_GPIO_AF(GPIO_Init->Alternate));
    /* Configure Alternate function mapped with the current IO */ 
    temp = GPIOx->AFR[position >> 3];
    CLEAR_BIT(temp, (uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;      
    SET_BIT(temp, (uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07) * 4));       
    GPIOx->AFR[position >> 3] = temp;
  }
  /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
  temp = GPIOx->MODER;
  CLEAR_BIT(temp, GPIO_MODER_MODER0 << (position * 2));   
  SET_BIT(temp, (GPIO_Init->Mode & ((uint32_t)0x00000003)) << (position * 2));
  GPIOx->MODER = temp;
  
  /* In case of Output or Alternate function mode selection */
  if ((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
      (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
  {
    /* Check the Speed parameter */
    assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
    /* Configure the IO Speed */
    temp = GPIOx->OSPEEDR; 
    CLEAR_BIT(temp, GPIO_OSPEEDER_OSPEEDR0 << (position * 2));
    SET_BIT(temp, GPIO_Init->Speed << (position * 2));
    GPIOx->OSPEEDR = temp;
    /* Configure the IO Output Type */
    temp = GPIOx->OTYPER;
    CLEAR_BIT(temp, GPIO_OTYPER_OT_0 << position) ;
    SET_BIT(temp, ((GPIO_Init->Mode & ((uint32_t)0x00000010)) >> 4) << position);
    GPIOx->OTYPER = temp;
  }
  /* Activate the Pull-up or Pull down resistor for the current IO */
  temp = GPIOx->PUPDR;
  CLEAR_BIT(temp, GPIO_PUPDR_PUPDR0 << (position * 2));
  SET_BIT(temp, (GPIO_Init->Pull) << (position * 2));
  GPIOx->PUPDR = temp;
}

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
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
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
    LL_TIM_DisableAllOutputs(htim->Instance);/*->BDTR &= ~(TIM_BDTR_MOE);*/
  }
  /* Disable the Peripheral */
  LL_TIM_DisableCounter(htim->Instance);
}

__STATIC_INLINE void SINGLE_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  /* Enable the Tx DMA channel */
  HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)pData, (uint32_t)&hspi->Instance->DR, Size);
  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(hspi);
  /* Enable Tx DMA Request */
  LL_SPI_EnableDMAReq_TX(hspi->Instance);
}

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
__STATIC_INLINE void SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
  __HAL_TIM_DISABLE_IT(htim, tim_it);
  /* Disable the Input Capture channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
  /* Disable the Peripheral */
  LL_TIM_DisableCounter(htim->Instance);
}

void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  SINGLE_TIM_IC_Stop_IT(htim,Channel,tim_it);
}

void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HardResetMode_TypeDef Mode)
{
  /* Reset the BIST index*/
  Ports[PortNum].BIST_index= 0;
  /* Reset the Bypass Bus Idle Flag */
  Ports[PortNum].BusIdleFlg = 0;
  /* Initialize State */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  RX_Init_Hvar(PortNum);  
}

HAL_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum)
{
  return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
