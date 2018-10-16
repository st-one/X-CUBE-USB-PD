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
#include "bmc.h"
#include "string.h"
#include "usbpd_timersserver.h"
#include "p-nucleo-usb001.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_bus.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PHY_HW_IF
  * @{
  */

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

#if !defined(USE_HAL_ADC)
/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

  /* Timeout values for ADC operations. */
  /* (calibration, enable settling time, disable settling time, ...)          */
  /* Values defined to be higher than worst cases: low clock frequency,       */
  /* maximum prescalers.                                                      */
  /* Unit: ms                                                                 */
  #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS  (   1U)
  #define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
  #define ADC_ENABLE_TIMEOUT_MS            (   1U)
  #define ADC_DISABLE_TIMEOUT_MS           (   1U)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
  #define ADC_CONVERSION_TIMEOUT_MS        (4000U)

  /* Delay between ADC end of calibration and ADC enable.                     */
  /* Delay estimation in CPU cycles: Case of ADC enable done                  */
  /* immediately after ADC calibration, ADC clock setting slow                */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
#endif /* USE_HAL_ADC */

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

USBPD_BoardVersionTypeDef BoardVersion;

CRC_HandleTypeDef   hcrc;                   /*!< Handle of CRC peripheral */
#if defined(USE_HAL_ADC)
ADC_HandleTypeDef   usbpdm1_hadc;           /*!< Handle of ADC peripheral */
#endif /* USE_HAL_ADC */
DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */

/* Variable containing ADC conversions results */
uint32_t   ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Handle for the ports inside @ref USBPD_HW_IF*/
USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/* Array of GPIO used by @ref P-NUCLEO-USB001 */
const USBPD_BSP_GPIOPins_TypeDef USBPDM1_GPIOs[USBPDM1_GPIOn] =
{
  USBPD_BSP_PIN(GPIOB, 2),   /* GPIO0 - POWCONN14 (ENABLE - PWREN_P1) */
  USBPD_BSP_PIN(GPIOB, 12),  /* GPIO1 - RP P0 (HRP_P0) */
  USBPD_BSP_PIN(GPIOB, 9),   /* GPIO2 - EN1 P0 (ENCC1_P0) */
  USBPD_BSP_PIN(GPIOC, 6),   /* GPIO3 - RD P1 (HRD_P1)  */
  USBPD_BSP_PIN(GPIOC, 3),   /* GPIO4 - EN2 P0 (ENCC2_P0) */
  USBPD_BSP_PIN(GPIOA, 8),   /* GPIO5 - POWCONN10 */
  USBPD_BSP_PIN(GPIOC, 7),   /* GPIO6 - RP P1 (HRP_P1 */
  USBPD_BSP_PIN(GPIOC, 8),   /* GPIO7 - RD P0 (HRD_P0) */
  USBPD_BSP_PIN(GPIOA, 15),  /* GPIO8 - EN1 P1 (ENCC1_P1) */
  USBPD_BSP_PIN(GPIOB, 5),   /* GPIO9 - EN2 P1 (ENCC2_P1) */
  USBPD_BSP_PIN(GPIOB, 8),   /* GPIO10 - POWCONN13 (ENABLE - PWREN_P0) */
  USBPD_BSP_PIN(GPIOD, 2),   /* GPIO11 - POWCONN15 (DISCHARGE- PWRDIS_P0) */
  USBPD_BSP_PIN(GPIOC, 14),  /* GPIO12 - POWCONN16 (DISCHARGE - PWRDIS_P1) */
#ifdef P_NUCLEO_USB001_GPIO13
  USBPD_BSP_PIN(GPIOC, 15),  /* GPIO13 - GREEN LED */
#endif
  USBPD_BSP_PIN(GPIOB, 0),   /* GPIO14 - POWCONN10 */
#ifdef P_NUCLEO_USB001_GPIO15
  USBPD_BSP_PIN(GPIOF, 1),   /* GPIO15 - ORANGE LED */
#endif
#ifndef P_NUCLEO_USB001_USE_USB2
  USBPD_BSP_PIN(GPIOA, 11),  /* GPIO16 - USBD0 (DM) */
  USBPD_BSP_PIN(GPIOA, 12)   /* GPIO17 - USBD1 (DP) */
#endif
};

/* Array of power selection pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_POWSELs[USBPDM1_POWSELn] =
{
  USBPD_BSP_PIN(GPIOB,7),   /* PORT0PW0 - POWCONN9 */
  USBPD_BSP_PIN(GPIOB,6),   /* PORT0PW1 - POWCONN11 */
  USBPD_BSP_PIN(GPIOC,1),   /* PORT1PW0 - POWCONN10 */
  USBPD_BSP_PIN(GPIOC,9),   /* PORT1PW1 - POWCONN12 */
};

/* Array of ADC CH pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_ADCs[USBPDM1_ADCn] =
{
#if defined(USE_HAL_ADC)
  USBPD_BSP_ADC(GPIOC, 4, ADC_CHANNEL_14),
  USBPD_BSP_ADC(GPIOA, 3, ADC_CHANNEL_3),
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOC, 5, ADC_CHANNEL_15),
  USBPD_BSP_ADC(GPIOA, 7, ADC_CHANNEL_7),
#endif
  USBPD_BSP_ADC(GPIOC, 0, ADC_CHANNEL_10)
#else
  USBPD_BSP_ADC(GPIOC, 4, LL_ADC_CHANNEL_14),   /* VBUS   PC4*/
  USBPD_BSP_ADC(GPIOA, 3, LL_ADC_CHANNEL_3),   /* IBUS   PA3*/
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOC, 5, LL_ADC_CHANNEL_15),
  USBPD_BSP_ADC(GPIOA, 7, LL_ADC_CHANNEL_7),
#endif
  USBPD_BSP_ADC(GPIOC, 0, LL_ADC_CHANNEL_10)    /* ADC4   POWER GOOD (POWCONN17 & POWCONN18)*/
#endif /* USE_HAL_ADC */
};

/* Array of RXD pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_RXD[USBPDM1_RXDn] =
{
#if defined(USE_HAL_ADC)
  USBPD_BSP_ADC(GPIOA, 0, ADC_CHANNEL_0), //P0.CC1
  USBPD_BSP_ADC(GPIOA, 5, ADC_CHANNEL_5), //P0.CC2
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOA, 2, ADC_CHANNEL_2), //P1.CC1
  USBPD_BSP_ADC(GPIOA, 4, ADC_CHANNEL_4), //P1.CC2
#endif
#else
  USBPD_BSP_ADC(GPIOA, 0, LL_ADC_CHANNEL_0),   /* P0 CC1 */
  USBPD_BSP_ADC(GPIOA, 5, LL_ADC_CHANNEL_5), /* P0 CC2 */
#if (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOA, 2, LL_ADC_CHANNEL_2), /* P1.CC1 */
  USBPD_BSP_ADC(GPIOA, 4, LL_ADC_CHANNEL_4), /* P1.CC2 */
#endif
#endif /* USE_HAL_ADC */
};

/* RXREF Pin used by @ref P-NUCLEO-USB001 */
#if defined(USE_HAL_ADC)
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_RXREF = USBPD_BSP_ADC(GPIOA, 1, ADC_CHANNEL_1);
#else
static const USBPD_BSP_GPIOPins_TypeDef USBPDM1_RXREF = USBPD_BSP_ADC(GPIOA, 1, LL_ADC_CHANNEL_1);
#endif /* USE_HAL_ADC */

/* Private function prototypes -----------------------------------------------*/

/* Common peripheral Init function*/
void USBPDM1_CRC_Init(void);
void USBPDM1_ADC_Init(void);
USBPD_BoardVersionTypeDef USBPD_HW_IF_GetBoardVersion(void);

/* Port related Init functions */
void USBPDM1_DigitalGPIO_Init(void);
void USBPDM1_DMA_Init(uint8_t PortNum);
void USBPDM1_COMP_SetCC(uint8_t PortNum, CCxPin_TypeDef ccx);
void USBPDM1_SPI_Init(uint8_t PortNum);
void USBPDM1_TX_TIM_Init(uint8_t PortNum);
void USBPDM1_RX_TIM_Init(uint8_t PortNum);
void USBPDM1_COMP_Init(uint8_t PortNum);
void USBPDM1_COUNTTIM_Init(uint8_t PortNum);

/* Pinout management functions */
__STATIC_INLINE void USBPDM1_DeInitRXD(uint8_t PortNum, CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_ReInitRXD(uint8_t PortNum, CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_SPI_Reset_TX_CC(uint8_t PortNum, CCxPin_TypeDef cc);
__STATIC_INLINE void USBPDM1_SetRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role);

/* Optimized functions */
__STATIC_INLINE void SINGLE_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin, uint32_t position);
__STATIC_INLINE void SINGLE_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init, uint32_t position);
#if defined(USE_HAL_TIM)
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
#else
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(uint8_t PortNum, uint32_t Channel);
#endif /* USE_HAL_TIM */
#if defined(USE_HAL_SPI)
__STATIC_INLINE void SINGLE_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
#endif /* USE_HAL_SPI */
#if defined(USE_HAL_TIM)
__STATIC_INLINE void SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
#else
__STATIC_INLINE void SINGLE_TIM_IC_Stop_IT(uint8_t PortNum, uint32_t Channel);
#endif /* USE_HAL_TIM */

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
  while (USBPD_TIM_IsExpired(TIM_PORT0_CRC) == 0)
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

/**
  * @brief  Configures P-NUCLEO-USB001 Digital GPIOs.
  * @retval None
  */
void USBPDM1_DigitalGPIO_Init(void)
{
  uint32_t gpio=0;
  GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  for (gpio = 0; gpio < USBPDM1_GPIOn; gpio++)
  {
    /* Configure the GPIO pin */
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[gpio].GPIO_Pin;
    HAL_GPIO_Init(USBPDM1_GPIOs[gpio].GPIOx, &GPIO_InitStruct);
  }

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  for (gpio = 0; gpio < USBPDM1_POWSELn; gpio++)
  {
    /* Configure the powsels pin */
    GPIO_InitStruct.Pin = USBPDM1_POWSELs[gpio].GPIO_Pin;

    HAL_GPIO_Init(USBPDM1_POWSELs[gpio].GPIOx, &GPIO_InitStruct);

    /* Turn the pin off */
    // USBPDM1_GPIO_Off((USBPDM1_GPIO_TypeDef)gpio);
  }

  /* Initialize the DRP pin*/
  GPIO_InitStruct.Pin = DRP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRP_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_SET);

  /* ENCC1 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC1_P0].GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC1_P0].GPIOx, &GPIO_InitStruct);
  /* ENCC2 should be PP */
  GPIO_InitStruct.Pin = USBPDM1_GPIOs[ENCC2_P0].GPIO_Pin;
  HAL_GPIO_Init(USBPDM1_GPIOs[ENCC2_P0].GPIOx, &GPIO_InitStruct);

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
      HAL_GPIO_DeInit(TX_CC1_GPIOPORT(PortNum), TX_CC1_PIN(PortNum));
      HAL_GPIO_DeInit(TX_CC2_GPIOPORT(PortNum), TX_CC2_PIN(PortNum));
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
      HAL_GPIO_DeInit(TX_CC1_GPIOPORT(PortNum), TX_CC1_PIN(PortNum));
      HAL_GPIO_DeInit(TX_CC2_GPIOPORT(PortNum), TX_CC2_PIN(PortNum));
      break;
  }
}

void USBPDM1_ADC_Init(void)
{
  uint8_t ch = 0;

  /*## Configuration of GPIO used by ADC channels ############################*/
  
#if defined(USE_HAL_ADC)
  ADC_ChannelConfTypeDef sConfig;
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

  for (ch = 0; ch < USBPDM1_ADCn; ch++)
  {
    sConfig.Channel = USBPDM1_ADCs[ch].ADCCH;
    HAL_ADC_ConfigChannel(&usbpdm1_hadc, &sConfig);
  }

  /* RXD Pins initialized as analog inputs */
  for (ch = 0; ch < USBPDM1_RXDn; ch++)
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
#else
  USBPDM1_ADCAnalogGPIO_Init();

  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable P_NUCLEO_USB001_ADC interruptions */
  NVIC_SetPriority(ADC1_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC1_COMP_IRQn);

  /*## Configuration of ADC ##################################################*/

  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/

  /* Enable ADC clock (core clock) */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC clock (conversion clock) common to several ADC instances */
    /* Note: On this STM32 serie, ADC common clock asynchonous prescaler      */
    /*       is applied to each ADC instance if ADC instance clock is         */
    /*       set to clock source asynchronous                                 */
    /*       (refer to function "LL_ADC_SetClock()" below).                   */
    // LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(P_NUCLEO_USB001_ADC), LL_ADC_CLOCK_ASYNC_DIV1);

    /* Set ADC measurement path to internal channels */
    // LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(P_NUCLEO_USB001_ADC), LL_ADC_PATH_INTERNAL_NONE);


  /*## Configuration of ADC hierarchical scope: multimode ####################*/

    /* Note: Feature not available on this STM32 serie */

  }


  /*## Configuration of ADC hierarchical scope: ADC instance #################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  //if (LL_ADC_IsEnabled(P_NUCLEO_USB001_ADC) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC clock (conversion clock) */
    LL_ADC_SetClock(P_NUCLEO_USB001_ADC, LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    /* Set ADC data resolution */
    // LL_ADC_SetResolution(P_NUCLEO_USB001_ADC, LL_ADC_RESOLUTION_12B);

    /* Set ADC conversion data alignment */
    // LL_ADC_SetResolution(P_NUCLEO_USB001_ADC, LL_ADC_DATA_ALIGN_RIGHT);

    /* Set ADC low power mode */
    // LL_ADC_SetLowPowerMode(P_NUCLEO_USB001_ADC, LL_ADC_LP_MODE_NONE);

    /* Set ADC channels sampling time */
    /* Note: On this STM32 serie, sampling time is common to all channels     */
    /*       of the entire ADC instance.                                      */
    /*       Therefore, sampling time is configured here under ADC instance   */
    /*       scope (not under channel scope as on some other STM32 devices    */
    /*       on which sampling time is channel wise).                         */
    /* Note: Considering interruption occurring after each ADC conversion     */
    /*       when ADC conversion is out of the analog watchdog window         */
    /*       selected (IT from ADC analog watchdog),                          */
    /*       select sampling time and ADC clock with sufficient               */
    /*       duration to not create an overhead situation in IRQHandler.      */
    if (BoardVersion == USBPD_BOARDVERSION_MB1257_B)
    {
      LL_ADC_SetSamplingTimeCommonChannels(P_NUCLEO_USB001_ADC, LL_ADC_SAMPLINGTIME_28CYCLES_5);
    }
    else
    {
      LL_ADC_SetSamplingTimeCommonChannels(P_NUCLEO_USB001_ADC, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    }
  }

  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on group regular.                                                  */
  //if ((LL_ADC_IsEnabled(P_NUCLEO_USB001_ADC) == 0)               ||
  //    (LL_ADC_REG_IsConversionOngoing(P_NUCLEO_USB001_ADC) == 0)   )
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(P_NUCLEO_USB001_ADC, LL_ADC_REG_TRIG_SOFTWARE);

    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(P_NUCLEO_USB001_ADC, LL_ADC_REG_TRIG_EXT_RISING);

    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(P_NUCLEO_USB001_ADC, LL_ADC_REG_CONV_CONTINUOUS);

    /* Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(P_NUCLEO_USB001_ADC, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* Set ADC group regular overrun behavior */
    LL_ADC_REG_SetOverrun(P_NUCLEO_USB001_ADC, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer is              */
    /*       not fully configurable: sequencer length and each rank           */
    /*       affectation to a channel are fixed by channel HW number.         */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerChannels()".                             */

    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(P_NUCLEO_USB001_ADC, LL_ADC_REG_SEQ_DISCONT_DISABLE);

    /* Set ADC group regular sequence: channel on rank corresponding to       */
    /* channel number.                                                        */
    for (ch = 0; ch < USBPDM1_ADCn; ch++)
    {
      LL_ADC_REG_SetSequencerChAdd(P_NUCLEO_USB001_ADC, USBPDM1_ADCs[ch].ADCCH);
    }

    /* RXD Pins initialized as analog inputs */
    for (ch = 0; ch < USBPDM1_RXDn; ch++)
    {
      LL_ADC_REG_SetSequencerChAdd(P_NUCLEO_USB001_ADC, USBPDM1_RXD[ch].ADCCH);
    }

    /* RXREF Pin initialized as analog inputs */
    LL_ADC_REG_SetSequencerChAdd(P_NUCLEO_USB001_ADC, USBPDM1_RXREF.ADCCH);

  }


  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/

  /* Note: Feature not available on this STM32 serie */

  /*## Configuration of ADC hierarchical scope: channels #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on either groups regular or injected.                              */
  if ((LL_ADC_IsEnabled(P_NUCLEO_USB001_ADC) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(P_NUCLEO_USB001_ADC) == 0)   )
  {
    /* Set ADC channels sampling time */
    /* Note: On this STM32 serie, sampling time is common to all channels     */
    /*       of the entire ADC instance.                                      */
    /*       See sampling time configured above, at ADC instance scope.       */

  }

  /*## Configuration of ADC transversal scope: analog watchdog ###############*/

  /* Note: On this STM32 serie, there is only 1 analog watchdog available.    */

  /* Set ADC analog watchdog: channels to be monitored */
  LL_ADC_SetAnalogWDMonitChannels(P_NUCLEO_USB001_ADC, __LL_ADC_ANALOGWD_CHANNEL_GROUP(LL_ADC_CHANNEL_15, LL_ADC_GROUP_REGULAR));

  /* Set ADC analog watchdog: thresholds */
  LL_ADC_ConfigAnalogWDThresholds(P_NUCLEO_USB001_ADC, MV2ADC(800), MV2ADC(400));

  /*## Configuration of ADC transversal scope: oversampling ##################*/

  /* Set ADC oversampling scope */
  // LL_ADC_SetOverSamplingScope(P_NUCLEO_USB001_ADC, LL_ADC_OVS_DISABLE);

  /* Set ADC oversampling parameters */
  // LL_ADC_ConfigOverSamplingRatioShift(P_NUCLEO_USB001_ADC, LL_ADC_OVS_RATIO_2, LL_ADC_OVS_SHIFT_NONE);

  /*## Configuration of ADC interruptions ####################################*/
  ///* Enable ADC analog watchdog 1 interruption */
  //LL_ADC_EnableIT_AWD1(P_NUCLEO_USB001_ADC);
#endif /* USE_HAL_ADC */
}

void USBPDM1_ADCAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;

  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  for (ch = 0; ch < USBPDM1_ADCn; ch++)
  {
    GPIO_InitStruct.Pin = USBPDM1_ADCs[ch].GPIO_Pin;
    HAL_GPIO_Init(USBPDM1_ADCs[ch].GPIOx, &GPIO_InitStruct);
  }
}

void USBPDM1_ADCAnalogGPIO_DeInit(void)
{
  uint8_t ch = 0;
  /* De-initialize GPIO pin of the selected ADC channel */

  for (ch = 0; ch < USBPDM1_ADCn; ch++)
  {
    HAL_GPIO_DeInit(USBPDM1_ADCs[ch].GPIOx, USBPDM1_ADCs[ch].GPIO_Pin);
  }
}

void USBPDM1_ADCDMA_Init(void)
{
#if defined(USE_HAL_ADC)
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
#else
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable DMA interruptions */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1); /* DMA IRQ lower priority than ADC IRQ */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /*## Configuration of DMA ##################################################*/
  /* Enable the peripheral clock of DMA */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Configure the DMA transfer */
  /*  - DMA transfer in circular mode to match with ADC configuration:        */
  /*    DMA unlimited requests.                                               */
  /*  - DMA transfer from ADC without address increment.                      */
  /*  - DMA transfer to memory with address increment.                        */
  /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
  /*    ADC resolution 12 bits.                                               */
  /*  - DMA transfer to memory by half-word to match with ADC conversion data */
  /*    buffer variable type: half-word.                                      */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_WORD            |
                        LL_DMA_PRIORITY_HIGH               );

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(P_NUCLEO_USB001_ADC, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&ADCxConvertedValues,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       ADCCONVERTEDVALUES_BUFFER_SIZE);

  /* Enable DMA transfer interruption: transfer complete */
  //LL_DMA_EnableIT_TC(DMA1,
  //                   LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: half transfer */
  //LL_DMA_EnableIT_HT(DMA1,
  //                   LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1,
                     LL_DMA_CHANNEL_1);

  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_1);
#endif /* USE_HAL_ADC */
}

void USBPDM1_ADCDMA_DeInit(void)
{
#if defined(USE_HAL_ADC)
  /* De-Initialize the DMA associated to the peripheral */
  if (usbpdm1_hadc.DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(usbpdm1_hadc.DMA_Handle);
  }
#else
  /* Disable the DMA transfer */
  LL_DMA_DisableChannel(DMA1,
                        LL_DMA_CHANNEL_1);
#endif /* USE_HAL_ADC */
}

void USBPDM1_COMPAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;

  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  /* RXD Pins initialized as analog inputs */
  for (ch = 0; ch < USBPDM1_RXDn; ch++)
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

  for (ch = 0; ch < USBPDM1_RXDn; ch++)
  {
    HAL_GPIO_DeInit(USBPDM1_RXD[ch].GPIOx, USBPDM1_RXD[ch].GPIO_Pin);
  }
  HAL_GPIO_DeInit(USBPDM1_RXREF.GPIOx, USBPDM1_RXREF.GPIO_Pin);
}

void USBPD_HW_IF_GlobalHwInit(void)
{
#if !defined(USE_HAL_ADC)
  __IO uint32_t wait_loop_index = 0U;
  //__IO uint32_t backup_setting_adc_dma_transfer = 0U;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
#endif /* USE_HAL_ADC */

  /* used for the PRL/PE timing */
  USBPD_TIM_Init();

  BoardVersion = USBPD_HW_IF_GetBoardVersion();

  /* Configure the ADCx peripheral */
  USBPDM1_ADC_Init();
  
#if defined(USE_HAL_ADC)
  /* Run the ADC calibration */
  HAL_ADCEx_Calibration_Start(&usbpdm1_hadc);

  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&usbpdm1_hadc, ADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE);
#else
  USBPDM1_ADCDMA_Init();

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  //if (LL_ADC_IsEnabled(P_NUCLEO_USB001_ADC) == 0)
  {
    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 serie: Calibration factor is           */
    /*       available in data register and also transfered by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in DMA destination address, DMA transfer must be disabled during */
    /*       calibration.                                                     */
    //backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(P_NUCLEO_USB001_ADC);
    LL_ADC_REG_SetDMATransfer(P_NUCLEO_USB001_ADC, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(P_NUCLEO_USB001_ADC);

    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(P_NUCLEO_USB001_ADC) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }

    /* Restore ADC DMA transfer request after calibration */
    //LL_ADC_REG_SetDMATransfer(P_NUCLEO_USB001_ADC, backup_setting_adc_dma_transfer);
    LL_ADC_REG_SetDMATransfer(P_NUCLEO_USB001_ADC, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(P_NUCLEO_USB001_ADC);

    /* Poll for ADC ready to convert */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

    while (LL_ADC_IsActiveFlag_ADRDY(P_NUCLEO_USB001_ADC) == 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }

    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }

  /*## Operation on ADC hierarchical scope: ADC group regular ################*/

  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of this feature is conditioned to     */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  //if ((LL_ADC_IsEnabled(P_NUCLEO_USB001_ADC) == 1)               &&
  //    (LL_ADC_IsDisableOngoing(P_NUCLEO_USB001_ADC) == 0)        &&
  //    (LL_ADC_REG_IsConversionOngoing(P_NUCLEO_USB001_ADC) == 0)   )
  {
    LL_ADC_REG_StartConversion(P_NUCLEO_USB001_ADC);
  }
  //else
  //{
  //  /* Error: ADC conversion start could not be performed */
  //}
#endif /* USE_HAL_ADC */

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
}

void USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role)
{
  /* Initialize default values for USBPD_HW_IF ports */
  Ports[PortNum].CCx        = CCNONE;
  Ports[PortNum].Lock       = HAL_UNLOCKED;
  Ports[PortNum].State      = HAL_USBPD_PORT_STATE_RESET;
  Ports[PortNum].ErrorCode  = 0;

  if (PortNum == USBPD_PORT_0)
  {
    Ports[USBPD_PORT_0].Instance   = USBPD_PORT_0;
    Ports[USBPD_PORT_0].pTxBuffPtr = (uint8_t *)TXBuffer0;
    Ports[USBPD_PORT_0].TxXferSize = DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t));
    Ports[USBPD_PORT_0].pRxBuffPtr = RXBuffer0;
    Ports[USBPD_PORT_0].pRxDataPtr = RXData0;
    Ports[USBPD_PORT_0].RxXferSize = PHY_MAX_RAW_SIZE;
  }

#if (USBPD_PORT_COUNT == 2)
  if (PortNum == USBPD_PORT_1)
  {
    Ports[USBPD_PORT_1].Instance   = USBPD_PORT_1;
    Ports[USBPD_PORT_1].pTxBuffPtr = (uint8_t *)TXBuffer1;
    Ports[USBPD_PORT_1].TxXferSize = DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t));
    Ports[USBPD_PORT_1].pRxBuffPtr = RXBuffer1;
    Ports[USBPD_PORT_1].pRxDataPtr = RXData1;
    Ports[USBPD_PORT_1].RxXferSize = PHY_MAX_RAW_SIZE;
  }
#endif

  /* Reset the BIST index*/
  Ports[PortNum].BIST_index = 0;

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
  USBPDM1_SetRole(PortNum, role);

  /* Initialize State and callbacks */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  Ports[PortNum].cbs = cbs;
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
  COMP_HandleTypeDef *phcomprx = &(Ports[PortNum].hcomprx);

  phcomprx->Instance               = RX_COMP(PortNum);
  phcomprx->Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  phcomprx->Init.InvertingInput    = RX_CC1_COMPCH(PortNum);
  phcomprx->Init.Output            = RX_COMPOUT(PortNum);
  phcomprx->Init.OutputPol         = COMP_OUTPUTPOL_NONINVERTED;
  phcomprx->Init.Hysteresis        = COMP_HYSTERESIS_HIGH;
  phcomprx->Init.WindowMode        = (RX_COMP(PortNum) == COMP2) ? COMP_WINDOWMODE_ENABLE : COMP_WINDOWMODE_DISABLE;
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
    /* if sink deinit the other RXD */
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
#if defined(USE_HAL_SPI)
  /* Get the peripheral handler variable */
  SPI_HandleTypeDef           *phspi = &(Ports[PortNum].hspitx);

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
#else
  /* Enable SPI and GPIO Clock */
  LL_SPI_CLK_ENABLE(PortNum);
  LL_SPI_GPIO_CLK_ENABLE(PortNum);
  LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Configure GPIO CLK pin */
  LL_GPIO_SetPinMode(LL_SPI_TX_CLK_PORT(PortNum),   LL_SPI_TX_CLK_PIN(PortNum), LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(LL_SPI_TX_CLK_PORT(PortNum), LL_SPI_TX_CLK_PIN(PortNum), LL_SPI_TX_CLK_SPI_GPIOAF(PortNum));
  LL_GPIO_SetPinSpeed(LL_SPI_TX_CLK_PORT(PortNum),  LL_SPI_TX_CLK_PIN(PortNum), LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(LL_SPI_TX_CLK_PORT(PortNum),   LL_SPI_TX_CLK_PIN(PortNum), LL_GPIO_PULL_NO);

  /* Configure SPI communication */
  LL_SPI_SetTransferDirection(TX_SPI(PortNum),LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(TX_SPI(PortNum), LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(TX_SPI(PortNum), LL_SPI_POLARITY_LOW);
  LL_SPI_SetTransferBitOrder(TX_SPI(PortNum), LL_SPI_LSB_FIRST);
  LL_SPI_SetDataWidth(TX_SPI(PortNum), LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(TX_SPI(PortNum), LL_SPI_NSS_SOFT);
  LL_SPI_SetRxFIFOThreshold(TX_SPI(PortNum), LL_SPI_RX_FIFO_TH_QUARTER);
  LL_SPI_SetMode(TX_SPI(PortNum), LL_SPI_MODE_SLAVE);

  /* Configure NVIC for DMA transfer complete/error interrupts */
  NVIC_SetPriority(TX_DMACHIRQ(PortNum), 0);
  NVIC_EnableIRQ(TX_DMACHIRQ(PortNum));

  /* Configure TX DMA functional parameters */
  LL_DMA_ConfigTransfer(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum),
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_VERYHIGH          |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);
#endif /* USE_HAL_SPI */
}

/* TX TIM init function */
void USBPDM1_TX_TIM_Init(uint8_t PortNum)
{
#if defined(USE_HAL_TIM)
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef           *phtimtx = &(Ports[PortNum].htimtx);
  TIM_OC_InitTypeDef           sConfigOC;
  uint32_t Period             = HAL_RCC_GetHCLKFreq() / BMC_TX_FREQ;

  phtimtx->Instance           = TX_TIM(PortNum);
  phtimtx->Init.Prescaler     = 0;
  phtimtx->Init.CounterMode   = TIM_COUNTERMODE_UP;
  phtimtx->Init.Period        = (Period - 1);
  phtimtx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(phtimtx);

  HAL_TIM_PWM_Init(phtimtx);

  sConfigOC.OCMode        = TIM_OCMODE_PWM1;
  sConfigOC.Pulse         = (Period / 2 - 1);
  sConfigOC.OCPolarity    = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode    = TIM_OCFAST_DISABLE;
  sConfigOC.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCIdleState   = TIM_OCIDLESTATE_SET;

  HAL_TIM_PWM_ConfigChannel(phtimtx, &sConfigOC, TX_TIMCH(PortNum));
#else
  uint32_t         Period = HAL_RCC_GetHCLKFreq() / BMC_TX_FREQ;

  /* TX TIM clock enable */
  TX_TIM_CLK_ENABLE(PortNum);

  /* TX TIM channel GPIO clock enable */
  TX_TIMCH_GPIO_CLK_ENABLE(PortNum);

  /* TX TIM channel GPIO configuration */
  LL_GPIO_SetPinMode(TX_TIM_GPIOPORT(port_num), TX_TIM_PIN(PortNum), LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TX_TIM_GPIOPORT(port_num), TX_TIM_PIN(PortNum), LL_GPIO_PULL_NO);
  LL_GPIO_SetPinSpeed(TX_TIM_GPIOPORT(port_num), TX_TIM_PIN(PortNum), LL_GPIO_SPEED_FREQ_LOW);

  if (TX_TIM_PIN(PortNum) <= LL_GPIO_PIN_7)
  {
    LL_GPIO_SetAFPin_0_7(TX_TIM_GPIOPORT(PortNum), TX_TIM_PIN(PortNum), TX_TIM_GPIOAF(PortNum));
  }
  else
  {
    LL_GPIO_SetAFPin_8_15(TX_TIM_GPIOPORT(PortNum), TX_TIM_PIN(PortNum), TX_TIM_GPIOAF(PortNum));
  }

  /* TX TIM time base unit configuration */
  LL_TIM_SetAutoReload(TX_TIM(PortNum), (Period - 1));

  /* TX TIM output channel configuration */
  LL_TIM_OC_SetMode(TX_TIM(PortNum), TX_TIMCH(PortNum), LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TX_TIM(PortNum), TX_TIMCH(PortNum));
  LL_TIM_OC_SetIdleState(TX_TIM(PortNum), TX_TIMCH(PortNum), LL_TIM_OCIDLESTATE_HIGH);
  TX_TIM_SET_COMPARE(PortNum, (Period / 2 - 1));
#endif /* USE_HAL_TIM */
}

/* RX TIM init function */
void USBPDM1_RX_TIM_Init(uint8_t PortNum)
{
#if defined(USE_HAL_TIM)
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef           *phtimrx = &(Ports[PortNum].htimrx);

  TIM_ClockConfigTypeDef   sClockSourceConfig;
  TIM_MasterConfigTypeDef   sMasterConfig;
  TIM_IC_InitTypeDef     sConfigIC;

  phtimrx->Instance           = RX_TIM(PortNum);
  phtimrx->Init.Prescaler     = (HAL_RCC_GetHCLKFreq() / (20 * BMC_TX_FREQ)) - 1 ;    // 3;
  phtimrx->Init.CounterMode   = TIM_COUNTERMODE_UP;
  phtimrx->Init.Period        = 255;
  phtimrx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  HAL_TIM_Base_Init(phtimrx);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(phtimrx, &sClockSourceConfig);

  HAL_TIM_IC_Init(phtimrx);

  /* Configure timer Channel according to CC line */
  sMasterConfig.MasterOutputTrigger =     TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode =         TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(phtimrx, &sMasterConfig);

  sConfigIC.ICPolarity  = TIM_TRIGGERPOLARITY_RISING;
  sConfigIC.ICSelection =   TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler =   TIM_ICPSC_DIV1;
  sConfigIC.ICFilter =       0;

  HAL_TIM_IC_ConfigChannel(phtimrx, &sConfigIC, RX_TIMCH(PortNum));
#else
  /* RX TIM clock enable */
  RX_TIM_CLK_ENABLE(PortNum);

  /* RX TIM DMA configuration */
  USBPDM1_RX_DMA_Init(PortNum);

  /* RX TIM time base unit configuration */
  LL_TIM_SetPrescaler(RX_TIM(PortNum), (HAL_RCC_GetHCLKFreq() / (20 * BMC_TX_FREQ)) - 1);
  LL_TIM_SetAutoReload(RX_TIM(PortNum), 255);
  LL_TIM_IC_SetActiveInput(RX_TIM(PortNum), RX_TIMCH(PortNum), LL_TIM_ACTIVEINPUT_DIRECTTI);
#endif /* USE_HAL_TIM */
}

/* COUNTTIM init function */
void USBPDM1_COUNTTIM_Init(uint8_t PortNum)
{
#if defined(USE_HAL_TIM)
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef *htimcountrx = &(Ports[PortNum].htimcountrx);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  htimcountrx->Instance                 = RX_COUNTTIM(PortNum);
  htimcountrx->Init.Prescaler           = ( HAL_RCC_GetHCLKFreq() / 1000000 ) - 1; /* 1us Resolution */
  htimcountrx->Init.CounterMode         = TIM_COUNTERMODE_UP;
  htimcountrx->Init.Period              = DMA_TIME_ELAPSED;
  htimcountrx->Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
  htimcountrx->Init.RepetitionCounter   = 0;
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
#else
  /* COUNT TIM clock enable */
  RX_COUNTTIM_CLK_ENABLE(PortNum);

  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(RX_COUNTTIM_IRQN(PortNum), RX_COUNTTIMIRQ_PRIO(PortNum), 0);
  HAL_NVIC_EnableIRQ(RX_COUNTTIM_IRQN(PortNum));

  /* COUNT TIM time base unit configuration */
  LL_TIM_SetPrescaler(RX_COUNTTIM(PortNum), (HAL_RCC_GetHCLKFreq() / 1000000) - 1);
  LL_TIM_SetAutoReload(RX_COUNTTIM(PortNum), DMA_TIME_ELAPSED);

  /* COUNT TIM output channel configuration */
  LL_TIM_OC_SetMode(RX_COUNTTIM(PortNum), TX_TIMCH(PortNum), LL_TIM_OCMODE_FROZEN);
  RX_COUNTTIM_SET_COMPARE(PortNum, DMA_TIME_COUNT_COMPARE);

  LL_TIM_ClearFlag_UPDATE(RX_COUNTTIM(PortNum));
#endif /* USE_HAL_TIM */

}

#if defined(USE_HAL_SPI)
void USBPDM1_TX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef *hdma_tx_spi = &(Ports[PortNum].hdmatx);
  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].hspitx.hdmatx = hdma_tx_spi;
  
  hdma_tx_spi->Instance                 = TX_DMACH(PortNum);
  hdma_tx_spi->Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx_spi->Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx_spi->Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx_spi->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx_spi->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx_spi->Init.Mode                = DMA_NORMAL;
  hdma_tx_spi->Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(hdma_tx_spi);

  __HAL_LINKDMA((&Ports[PortNum].hspitx), hdmatx, (*hdma_tx_spi));

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(TX_DMACHIRQ(PortNum), TX_DMACHIRQ_PRIO(PortNum), 0);
}
#endif /* USE_HAL_SPI */

void USBPDM1_RX_EnableInterrupt(uint8_t PortNum)
{
  /* Set the port state to waiting */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
#if defined(USE_HAL_TIM)
  __HAL_TIM_ENABLE(&(Ports[PortNum].htimrx));

  /* Add enable the DMA */
  /* Enable the Rx interrupt if cbs are initialized; this mean that this is a PD capable port */
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  HAL_TIM_IC_Start_IT(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
#else
  LL_TIM_EnableCounter(RX_TIM(PortNum));
  HAL_DMA_Start(&(Ports[PortNum].hdmarx), (uint32_t)&((RX_TIM(PortNum))->CCR1), (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  LL_TIM_EnableIT_CC1(RX_TIM(PortNum));
  LL_TIM_CC_EnableChannel(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1);
#endif /* USE_HAL_TIM */
}

void USBPDM1_RX_DisableInterrupt(uint8_t PortNum)
{
  /* The port is ready to transmit */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;

#if defined USE_HAL_TIM
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
#else
  LL_TIM_DisableCounter(RX_TIM(PortNum));

  SINGLE_TIM_IC_Stop_IT(PortNum, RX_TIMCH(PortNum));
  /* Stop DMA transfers */
  LL_TIM_CC_DisableChannel(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1);
  LL_TIM_DisableDMAReq_CC1(RX_TIM(PortNum));
  /* stop the TIM counter for the selected port */
  SINGLE_TIM_OC_Stop_IT(PortNum, RX_COUNTTIMCH(PortNum));
  /* Disable the TIM Update interrupt */
  LL_TIM_DisableIT_UPDATE(RX_COUNTTIM(PortNum));
  /* Reset the counter */
  LL_TIM_SetCounter(RX_COUNTTIM(PortNum), 0);
#endif /* USE_HAL_TIM */

  /* Add disable the DMA */
  __HAL_DMA_DISABLE(&(Ports[PortNum].hdmarx));
}

void USBPDM1_RX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef *hdma_rx_tim = &Ports[PortNum].hdmarx;

#if defined(USE_HAL_TIM)
  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].htimrx.hdma[TIM_DMA_ID_CC1] = hdma_rx_tim;
#endif /* USE_HAL_TIM */

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

#if defined(USE_HAL_TIM)
  /* Several peripheral DMA handle pointers point to the same DMA handle.
  Be aware that there is only one channel to perform all the requested DMAs. */
  __HAL_LINKDMA(&(Ports[PortNum].htimrx), hdma[TIM_DMA_ID_CC1], (*hdma_rx_tim));
#endif /* USE_HAL_TIM */
}

void USBPDM1_RX_DMA_Deinit(uint8_t PortNum)
{
#if defined(USE_HAL_TIM)
  /* Peripheral DMA DeInit*/
  HAL_DMA_DeInit(Ports[PortNum].htimrx.hdma[TIM_DMA_ID_CC1]);
#else
  HAL_DMA_DeInit(&(Ports[PortNum].hdmarx));
#endif
}

USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize)
{
  /* Check if the port is yet receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
  {
    return USBPD_BUSY;
  }

  USBPD_StatusTypeDef ret = USBPD_ERROR;

  uint16_t size = 0;
  uint16_t *pTxBuffer = (uint16_t *)Ports[PortNum].pTxBuffPtr;
  uint16_t *pTxDataBuffer = pTxBuffer + (TX_PREAMBLE_SIZE / 2); /* pointer of the first data, after the preamble */
  uint16_t sTxDataBufferBitsize = 0;                            /* size of data (SOP ... EOP) */
  uint16_t sStartupValue = 0;
  uint16_t sMaxTXDataBufferSize = TX_BUFFER_LEN * 4 - TX_PREAMBLE_SIZE; /* bytes */
  uint8_t nLastBit = 0x00;
  uint16_t nTotalBitsize = 0x00;

  memset((uint8_t *)pTxBuffer, 0x00, TX_BUFFER_SIZE);
  memset((uint8_t *)pTxBuffer, TX_PREAMBLE_BMC_CODED, TX_PREAMBLE_SIZE); /* preamble length */

  sStartupValue = *(uint16_t *)(pTxDataBuffer - 1); /* get the previous value (it is preamble) */
  ret = BMC_MakeCoding(pBuffer, Bitsize, pTxDataBuffer, &sTxDataBufferBitsize, sMaxTXDataBufferSize, sStartupValue, &nLastBit);

  /* Add last edge */
  nTotalBitsize = TX_PREAMBLE_SIZE * 8 + sTxDataBufferBitsize;
  if (nLastBit == 0x00)
  {
    uint16_t *pLastItem = (uint16_t *) & (pTxBuffer[nTotalBitsize / 16]);
    uint16_t edgeBit = nTotalBitsize % 16;
    *pLastItem |= 3 << edgeBit;
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
  {
    return ret;
  }

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
  ret = USBPD_OK;

  if (Ports[PortNum].params->PE_SwapOngoing == 0)
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

    if (Ports[PortNum].resistor)
    {
      USBPDM1_DeAssertRp(PortNum);
    }

    /* Start the timer clocking the SPI */
  /* Start the timer clocking the SPI */
#if defined(USE_HAL_TIM)
  HAL_TIM_PWM_Start(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));
#else
  LL_TIM_CC_EnableChannel(TX_TIM(PortNum), TX_TIMCH(PortNum));
  if (IS_TIM_BREAK_INSTANCE(TX_TIM(PortNum)))
  {
    LL_TIM_EnableAllOutputs(TX_TIM(PortNum));
  }
  LL_TIM_EnableCounter(TX_TIM(PortNum));
#endif

#if defined(USE_HAL_SPI)
    /* Start transmission */
    SINGLE_SPI_Transmit_DMA(&(Ports[PortNum].hspitx), Ports[PortNum].pTxBuffPtr, size);
#else
   /* Configure SPI DMA TX Addresses parameters */
   LL_DMA_ConfigAddresses(TX_DMA(PortNum),
                         TX_DMACH_NUMBER(PortNum),
                         (uint32_t)Ports[PortNum].pTxBuffPtr,
                         LL_SPI_DMA_GetRegAddr(TX_SPI(PortNum)),
                         LL_DMA_GetDataTransferDirection(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum)));

    /* Set SPI DMA transfer lenght */
    LL_DMA_SetDataLength(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum), size);

    /* Set SPI DMA parity */
    if ((size & 0x1U) == 0U)
    {
       LL_SPI_SetDMAParity_TX(TX_SPI(PortNum), LL_SPI_DMA_PARITY_EVEN);
    }
    else
    {
      LL_SPI_SetDMAParity_TX(TX_SPI(PortNum), LL_SPI_DMA_PARITY_ODD);
    }

    /* Enable DMA Interrupt TC: Transfer Complete */
    LL_DMA_EnableIT_TC(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum));

    /* Enable SPI DMA TX Request Interrupt */
    LL_SPI_EnableDMAReq_TX(TX_SPI(PortNum));

    /* Enable SPI */
    LL_SPI_Enable(TX_SPI(PortNum));

    /* Enable DMA Channels */
    LL_DMA_EnableChannel(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum));
#endif /* USE_HAL_SPI */
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
  * @param  PortNum Specify the port to check.
  * @retval USBPD_OK => bus idle, USBPD_BUSY => bus busy, error and timeout is not allowed
  */

uint8_t ADC_AWDEvent = 0;
uint32_t ADC_AWDEventCount = 0;
uint8_t CCPortIndex = 0;
uint32_t CCPortADCCh = 0;
uint32_t CCAnalogValue = 0;
uint16_t ICDiff = 0;
USBPD_StatusTypeDef HW_IF_CheckBusIdle(uint8_t PortNum)
{
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
  if (Ports[PortNum].BusIdleFlg == 1)
  {
    return USBPD_OK;
  }

  USBPD_StatusTypeDef ret = USBPD_OK;
#if !defined(USE_HAL_ADC)
  uint8_t adc_conversion_stopped = 0; /* Flag to memorize that ADC group regular conversion has been stopped and must be restored */
#endif /* USE_HAL_ADC */

  /****** Check edges ******/
#if defined(USE_HAL_TIM)
  /* configure the input capture timer on both edge to get possible transitions in CCx line */
  Ports[PortNum].htimrx.Instance->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP; /* BOTHEDGE */

  /* start the counting of the possible transitions in CCx line */
  HAL_DMA_Start(Ports[PortNum].htimrx.hdma[RX_TIM_DMA_ID_CC(PortNum)], (uint32_t)&Ports[PortNum].htimrx.Instance->CCR1, (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  __HAL_TIM_ENABLE_DMA(&(Ports[PortNum].htimrx), RX_TIM_DMA_CC(PortNum));
  HAL_TIM_IC_Start(&(Ports[PortNum].htimrx), RX_TIMCH(PortNum));
#else
  LL_TIM_IC_SetPolarity(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_BOTHEDGE);
  HAL_DMA_Start(&(Ports[PortNum].hdmarx), (uint32_t)&((RX_TIM(PortNum))->CCR1), (uint32_t)Ports[PortNum].pRxBuffPtr, PHY_MAX_RAW_SIZE);
  LL_TIM_EnableDMAReq_CC1(RX_TIM(PortNum));
  LL_TIM_CC_EnableChannel(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(RX_TIM(PortNum));
#endif

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
  CCPortIndex = CC_INDEX(PortNum, Ports[PortNum].CCx);
  CCPortADCCh = CC_ADC_CHANNEL(PortNum, Ports[PortNum].CCx);

#if defined(USE_HAL_ADC)
  /* enable analog watchdog in CCx line to recognize if noise or transmission */
  usbpdm1_hadc.Instance->ISR |= ADC_FLAG_AWD; /* reset the AWD events */
  usbpdm1_hadc.Instance->CFGR1 &= ~(ADC_CFGR1_AWD1CH_Msk); /* set 0 the channels in the register */
  usbpdm1_hadc.Instance->CFGR1 |= ADC_ANALOGWATCHDOG_SINGLE_REG | (CCPortADCCh << ADC_CFGR1_AWD1CH_Pos); /* set the AWD single channel and set the channel */
#else
  /* enable analog watchdog in CCx line to recognize if noise or transmission */
  if (LL_ADC_REG_IsConversionOngoing(P_NUCLEO_USB001_ADC) == 1)
  {
    /* Stop ADC group regular conversion before modifying AWD parameters */
//    LL_ADC_REG_StopConversion(P_NUCLEO_USB001_ADC);

    adc_conversion_stopped = 1;
  }

  /* Clear flag ADC analog watchdog 1 */
  LL_ADC_ClearFlag_AWD1(P_NUCLEO_USB001_ADC);

  /* Set ADC analog watchdog: channels to be monitored */
  LL_ADC_SetAnalogWDMonitChannels(P_NUCLEO_USB001_ADC, __LL_ADC_ANALOGWD_CHANNEL_GROUP(CCPortADCCh, LL_ADC_GROUP_REGULAR));

  if(adc_conversion_stopped != 0)
  {
    /* Start ADC group regular conversion */
//    LL_ADC_REG_StartConversion(P_NUCLEO_USB001_ADC);
  }
#endif /* USE_HAL_ADC */

  /* wait 12-20us */
  USBPD_TIM_Start((PortNum == USBPD_PORT_0 ? TIM_PORT0_CRC : TIM_PORT1_CRC), 20);
  while (USBPD_TIM_IsExpired((PortNum == USBPD_PORT_0 ? TIM_PORT0_CRC : TIM_PORT1_CRC)) == 0)
  {
  }

  /* Get the bus value */
  CCAnalogValue = ADCxConvertedValues[CCPortIndex];

#if defined(USE_HAL_ADC)
  /* Get AWD Event */
  ADC_AWDEvent = usbpdm1_hadc.Instance->ISR & ADC_FLAG_AWD;

  /* After 12 - 20 us Stop the AWD */
  usbpdm1_hadc.Instance->CFGR1 &= ~ADC_ANALOGWATCHDOG_SINGLE_REG;
#else
  /* Get AWD Event */
  /* Get flag ADC analog watchdog 1 */
  ADC_AWDEvent = LL_ADC_IsActiveFlag_AWD1(P_NUCLEO_USB001_ADC);

  /* After 12 - 20 us Stop the AWD */
  /* Set ADC analog watchdog: channels to be monitored */
  LL_ADC_SetAnalogWDMonitChannels(P_NUCLEO_USB001_ADC, LL_ADC_AWD_DISABLE);
#endif /* USE_HAL_ADC */

  /* Evaluate number RX Events */
  ICDiff = PHY_MAX_RAW_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR;

  if (ADC_AWDEvent == 0)
  {
    ADC_AWDEventCount++;
  }

  /* Check Bus Condition */
  if ((ADC_AWDEvent > 0) && ((ICDiff >= BUSCHECK_IC_NUMBER) || (CCAnalogValue <= BUSCHECK_THRESH_LOW)))
  {
    ret = USBPD_BUSY;
  }

  /* set the interrupt of the timer to get rising events */
#if defined(USE_HAL_TIM)
  Ports[PortNum].htimrx.Instance->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); /* RISING */
#else
  LL_TIM_IC_SetPolarity(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
#endif

  return ret;
}

void USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  /* BIST Carrier mode flag set */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BIST;

  /* Fill the buffer with the pattern to be sent */
  memset(Ports[PortNum].pTxBuffPtr, 0xB4, TX_BUFFER_LEN * 4);

  /* start a circular DMA transfer */
  USBPDM1_Set_DMA_Circular_Mode(PortNum);

  /* Disable the Rx process */
  USBPDM1_RX_DisableInterrupt(PortNum);
  /* Enables the TX TC Interrupt */
  HAL_NVIC_EnableIRQ(TX_DMACHIRQ(PortNum));
  /* Set the state to busy*/
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BIST;
  /* Start the timer clocking the SPI */
#if defined(USE_HAL_TIM)
  HAL_TIM_PWM_Start(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));
#else
  LL_TIM_CC_EnableChannel(TX_TIM(PortNum), TX_TIMCH(PortNum));
  LL_TIM_EnableCounter(TX_TIM(PortNum));
  if (IS_TIM_BREAK_INSTANCE(TX_TIM(PortNum)))
  {
    LL_TIM_EnableAllOutputs(TX_TIM(PortNum));
  }
#endif
  /* Set the pin to be used for transmission by SPI */
  USBPDM1_SPI_Set_TX_CC(PortNum, Ports[PortNum].CCx);
  /* Set the correct GPIOs */
  USBPDM1_DeInitRXD(PortNum, Ports[PortNum].CCx);

#if defined(USE_HAL_SPI)
  /* Start transmission */
  HAL_SPI_Transmit_DMA(&(Ports[PortNum].hspitx), (uint8_t *)(Ports[PortNum].pTxBuffPtr), TX_BUFFER_LEN * 4);
#else
  /* Configure SPI DMA TX Addresses parameters */
  LL_DMA_ConfigAddresses(TX_DMA(PortNum),
                         TX_DMACH_NUMBER(PortNum),
                         (uint32_t)Ports[PortNum].pTxBuffPtr,
                         LL_SPI_DMA_GetRegAddr(TX_SPI(PortNum)),
                         LL_DMA_GetDataTransferDirection(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum)));

  /* Set SPI DMa transfer lenght */
  LL_DMA_SetDataLength(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum), (TX_BUFFER_LEN * 4));

  /* Set SPI DMA parity */
  LL_SPI_SetDMAParity_TX(TX_SPI(PortNum), LL_SPI_DMA_PARITY_EVEN);

  /* Enable DMA Interrupt TC: Transfer Complete */
  LL_DMA_EnableIT_TC(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum));

  /* Enable DMA TX Interrupt */
  LL_SPI_EnableDMAReq_TX(TX_SPI(PortNum));

  /* Enable SPI */
  LL_SPI_Enable(TX_SPI(PortNum));

  /* Enable DMA Channels */
  LL_DMA_EnableChannel(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum));
#endif /* USE_HAL_SPI */
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
#if defined(USE_HAL_SPI)
  /* Wait for FIFO empty */
  while (LL_SPI_GetTxFIFOLevel(Ports[PortNum].hspitx.Instance) != LL_SPI_TX_FIFO_EMPTY);
  /* Wait for BUSY flag */
  while (LL_SPI_IsActiveFlag_BSY(Ports[PortNum].hspitx.Instance));
  /* Here the SPI has completed the transmission*/
  /* Stop SPI */
  LL_SPI_Disable(Ports[PortNum].hspitx.Instance); /*->CR1 &= 0xFFBF;*/
#else
  /* Wait for FIFO empty */
  while (LL_SPI_GetTxFIFOLevel(TX_SPI(PortNum)) != LL_SPI_TX_FIFO_EMPTY);
  /* Wait for BUSY flag */
  while (LL_SPI_IsActiveFlag_BSY(TX_SPI(PortNum)));
  /* Here the SPI has completed the transmission*/
  /* Stop SPI */
  LL_SPI_Disable(TX_SPI(PortNum)); /*->CR1 &= 0xFFBF;*/
#endif /* USE_HAL_SPI */
  /* Configure RXD as RX pin */
  USBPDM1_ReInitRXD(PortNum, Ports[PortNum].CCx);
  /* Put SPI MISO pin in high impedence */
  USBPDM1_SPI_Reset_TX_CC(PortNum, Ports[PortNum].CCx);

  if (Ports[PortNum].resistor)
  {
    USBPDM1_AssertRp(PortNum);
  }

#if defined(USE_HAL_SPI)
  /* Clean DMA flags */
  HAL_DMA_IRQHandler(&Ports[PortNum].hdmatx);
  /* Disable TX interrupt */
  HAL_NVIC_DisableIRQ(TX_DMACHIRQ(PortNum));

#else
  /* Disable DMA TX Interrupt */
  LL_SPI_DisableDMAReq_TX(TX_SPI(PortNum));
  /* Disable TX interrupt */
  NVIC_DisableIRQ(TX_DMACHIRQ(PortNum));
  /* Disable DMA Channels */
  LL_DMA_DisableChannel(TX_DMA(PortNum), TX_DMACH_NUMBER(PortNum));
#endif /* USE_HAL_SPI */
  /* Stop the timer clocking the SPI */
  /* Start the timer clocking the SPI */
#if defined(USE_HAL_TIM)
  HAL_TIM_PWM_Stop(&(Ports[PortNum].htimtx), TX_TIMCH(PortNum));
#else
  LL_TIM_CC_DisableChannel(TX_TIM(PortNum), TX_TIMCH(PortNum));
  if (IS_TIM_BREAK_INSTANCE(TX_TIM(PortNum)))
  {
    LL_TIM_DisableAllOutputs(TX_TIM(PortNum));
  }
  LL_TIM_DisableCounter(TX_TIM(PortNum));
#endif

  /* Check if BIST TX Done */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BIST)
  {
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_RESET;
    /* Evaluate callback*/
    if ((Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted != NULL)) /*&& (bitsize>0) )*/
    {
      Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted(PortNum, USBPD_BIST_CARRIER_MODE2);
    }
  }

  /* Enable RX Interrupt */
  USBPDM1_RX_EnableInterrupt(PortNum);

  if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum);
  }
}

__STATIC_INLINE void USBPDM1_DeInitRXD(uint8_t PortNum, CCxPin_TypeDef cc)
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

uint8_t USBPDM1_IsAnalog(uint8_t PortNum, CCxPin_TypeDef cc);

uint8_t USBPDM1_IsAnalog(uint8_t PortNum, CCxPin_TypeDef cc)
{
  uint8_t _res = USBPD_FALSE;

  switch (cc)
  {
    case CC1:
      if (LL_GPIO_MODE_ANALOG == LL_GPIO_GetPinMode(RX_CC1_GPIOPORT(PortNum), RX_CC1_PIN(PortNum)))
      {
        _res = USBPD_TRUE;
      }
      break;
    case CC2:
      if (LL_GPIO_MODE_ANALOG == LL_GPIO_GetPinMode(RX_CC2_GPIOPORT(PortNum), RX_CC2_PIN(PortNum)))
      {
        _res = USBPD_TRUE;
      }
      break;
    default:
      break;
  }
  return _res;
}

__STATIC_INLINE void USBPDM1_ReInitRXD(uint8_t PortNum, CCxPin_TypeDef cc)
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
      SINGLE_GPIO_DeInit(RX_CC1_GPIOPORT(PortNum), RX_CC1_PIN(PortNum), RX_CC1_PIN_POSITION(PortNum));
      SINGLE_GPIO_Init(RX_CC1_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC1_PIN_POSITION(PortNum));
      break;
    case CC2:
      GPIO_InitStruct.Pin = RX_CC2_PIN(PortNum);
      SINGLE_GPIO_DeInit(RX_CC2_GPIOPORT(PortNum), RX_CC2_PIN(PortNum), RX_CC2_PIN_POSITION(PortNum));
      SINGLE_GPIO_Init(RX_CC2_GPIOPORT(PortNum), &GPIO_InitStruct, RX_CC2_PIN_POSITION(PortNum));
      break;
    default:
      break;
  }
}

void RX_Init_Hvar(uint8_t PortNum)
{
  /* Decoding variables */
  FiveBitCodingVar_TypeDef *hvar = &(Ports[PortNum].decfields);

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

  uint16_t last_index = 0;
  uint8_t  curr_bit = 0;
  uint8_t  diff_noise;
#if defined(USE_HAL_TIM)
#else
  uint32_t counter;
#endif

  /* Get the peripheral handler */
  FiveBitCodingVar_TypeDef *hvar = &(Ports[PortNum].decfields);
  /* RX2 is the pointer to raw data buffer */
  uint8_t *RX2 = Ports[PortNum].pRxBuffPtr; //DMA data -- input buffer
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
        {
          break;
        }

        /* the cpu resource is finished */
#if defined(USE_HAL_TIM)
        if (Phase == 0 && Ports[PortNum].htimcountrx.Instance->CNT >= DMA_TIME_TASK)
#else
        counter = LL_TIM_GetCounter(RX_COUNTTIM(PortNum));
        if ((Phase == 0) && (counter >= DMA_TIME_TASK))
#endif /* USE_HAL_TIM */
        {
          break;
        }

        /* calculate the difference with a safe threshold */
        diff_noise = (RX2[hvar->curr_indx] - RX2[hvar->curr_indx - 1] + PARAM_RX_OFFSET);

        /* increment the index */
        hvar->curr_indx++;

        /* to avoid glitch in presence of noise */
        if (diff_noise <= (3 + PARAM_RX_OFFSET))
        {
          continue;
        }

        /* calculate the current bit diff/30 and not */
        curr_bit = !((diff_noise >> 5) & 0x01);

        /* increment the index if detect a 1 */
        hvar->curr_indx += curr_bit;

        /* verify the end of the preamble checking for two consecutive bit equal.
        * Shift first five ones */
        if ((curr_bit == hvar->prev_bit) && (hvar->curr_indx > 10))
        {
          /* set the preamble flag to 1 */
          hvar->preamble = 1;
          hvar->temp_data = 0;
          if (curr_bit == 0)
          {
            /* set 0 to position  0 to recover the first 0 */
            hvar->temp_data |= curr_bit;
            hvar->j = 1;
          }
          /* set next bit */
          hvar->temp_data |= (curr_bit << hvar->j);
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
        {
          break;
        }

        /* the cpu resource is finished */
#if defined(USE_HAL_TIM)
        if (Phase == 0 && Ports[PortNum].htimcountrx.Instance->CNT >= DMA_TIME_TASK && hvar->j > 5)
#else
        counter = LL_TIM_GetCounter(RX_COUNTTIM(PortNum));
        if ((Phase == 0) && (counter >= DMA_TIME_TASK) && (hvar->j > 5))
#endif /* USE_HAL_TIM */
        {
          break;
        }

        /* calculate the difference with a safe threshold */
        diff_noise = (RX2[hvar->curr_indx] - RX2[hvar->curr_indx - 1] + PARAM_RX_OFFSET);

        /* increment the index */
        hvar->curr_indx++;

        /* to avoid glitch in presence of noise */
        if (diff_noise <= (3 + PARAM_RX_OFFSET))
        {
          continue;
        }

        /* calculate the current bit diff/30 and not */
        curr_bit = !((diff_noise >> 5) & 0x01);

        /* increment the index if detect a 1 */
        hvar->curr_indx += curr_bit;

        /* write next bits */
        hvar->temp_data |= (curr_bit << hvar->j);
        hvar->j++;

        if (hvar->j == 10)
        {
          Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, hvar->temp_data);
          hvar->j = 0;
          hvar->k++; /* only used to check the overflow */
          hvar->temp_data = 0;

          /* re-calculate the last index */
          //last_index = (PHY_MAX_RAW_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR) + 1;
        }
        if (hvar->j == 5 && hvar->temp_data == 0x0D) //EOP
        {
          Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, hvar->temp_data);
          hvar->exed_flag = 2;
        }
      }
    }
  }
}

#if defined(USE_HAL_TIM)
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
#else
void DelayElapsedCallback(uint8_t PortNum)
#endif /* USE_HAL_TIM */
{
#if defined(USE_HAL_TIM)
  uint8_t PortNum;
#endif /* USE_HAL_TIM */
  uint32_t bitsize = 0;
  uint32_t tim_count = 0;
  uint32_t dma_count = 0;
  uint32_t dma_count2 = 0;
#if defined(USE_HAL_TIM)
#else
  uint32_t counter;
#endif

#if defined(USE_HAL_TIM)
#if (USBPD_PORT_COUNT==1)
  PortNum = 0;
#elif (USBPD_PORT_COUNT==2)
  PortNum = ((htim->Instance == RX_COUNTTIM(0)) ? 0 : 1);
#endif /* RX TIMER IDENTIFIED */
#endif /* USE_HAL_TIM */

  /* The message has been transmitted if non change on DMA->CNDTR*/
  Ports[PortNum].decfields.DMA_count = Ports[PortNum].hdmarx.Instance->CNDTR;

  /* perform a decoding task in any case */
  //GPIOA->BSRR = GPIO_PIN_8;
  HW_IF_DecodingTask(PortNum, 0);
  //GPIOA->BRR = GPIO_PIN_8;

  dma_count = 0;
  if (Ports[PortNum].decfields.exed_flag == 0)
  {
#if defined(USE_HAL_TIM)
    /* get the tim count with a safe check */
    tim_count = MIN(Ports[PortNum].htimcountrx.Instance->CNT, DMA_TIME_THRESHOLD1);

    /* waiting 5us if there is enough time */
    while (Ports[PortNum].htimcountrx.Instance->CNT < (tim_count + DMA_TIME_DURATION));

    /* wait few us and so check again the dma count */
    tim_count = MIN(Ports[PortNum].htimcountrx.Instance->CNT, DMA_TIME_THRESHOLD2);
    dma_count = Ports[PortNum].hdmarx.Instance->CNDTR;

    //GPIOA->BRR = GPIO_PIN_8;
    while (Ports[PortNum].htimcountrx.Instance->CNT < tim_count + DMA_TIME_DURATION);
#else
    /* get the tim count with a safe check */
    counter = LL_TIM_GetCounter(RX_COUNTTIM(PortNum));
    tim_count = MIN(counter, DMA_TIME_THRESHOLD1);

    /* waiting 5us if there is enough time */
    do
    {
      counter = LL_TIM_GetCounter(RX_COUNTTIM(PortNum));
    } while (counter < (tim_count + DMA_TIME_DURATION));

    /* wait few us and so check again the dma count */
    tim_count = MIN(counter, DMA_TIME_THRESHOLD2);
    dma_count = Ports[PortNum].hdmarx.Instance->CNDTR;

    do
    {
      counter = LL_TIM_GetCounter(RX_COUNTTIM(PortNum));
    } while (counter < (tim_count + DMA_TIME_DURATION));
#endif /* USE_HAL_TIM */

    //GPIOA->BSRR = GPIO_PIN_8;
    dma_count2 = Ports[PortNum].hdmarx.Instance->CNDTR;
  }

  /* check if the dma_count doesn't change */
#if defined(USE_HAL_TIM)
  if (((dma_count2 == dma_count) || (Ports[PortNum].decfields.exed_flag > 0)))
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
      if ((Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed != NULL) && (bitsize > 0))
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
#else
  if (((dma_count2 == dma_count) || (Ports[PortNum].decfields.exed_flag > 0)))
  {
    /* PC04 A8 reset : Rx start */
    GPIOA->BRR = GPIO_PIN_8;

    /* PC01 A8 set : start of the transmission */
    //GPIOA->BSRR = GPIO_PIN_8;
    Ports[PortNum].decfields.DMA_count = dma_count;
    /* Stop the TIM DMA transfers */
    LL_TIM_DisableDMAReq_CC1(RX_TIM(PortNum));
    LL_TIM_CC_DisableChannel(RX_TIM(PortNum), RX_TIMCH(PortNum));
    LL_TIM_DisableCounter(RX_TIM(PortNum));

    /* DMA Abort to execute the DMA Deinit */
    HAL_DMA_Abort(&Ports[PortNum].hdmarx);

    /* stop the TIM counter for the selected port */
    LL_TIM_SetCounter(RX_COUNTTIM(PortNum), 0);

    /* stop the TIM counter for the selected port */
    LL_TIM_CC_DisableChannel(RX_COUNTTIM(PortNum), RX_COUNTTIMCH(PortNum));
    if (IS_TIM_BREAK_INSTANCE(RX_COUNTTIM(PortNum)))
    {
      LL_TIM_DisableAllOutputs(RX_COUNTTIM(PortNum));
    }
    LL_TIM_DisableCounter(RX_COUNTTIM(PortNum));

    /* Disable the TIM Update interrupt */
    LL_TIM_DisableIT_UPDATE(RX_COUNTTIM(PortNum));

    /* start Data reading and decoding in the final phase (1)  */
    if (Ports[PortNum].decfields.exed_flag == 0)
    {
      HW_IF_DecodingTask(PortNum, 1);
    }

    /* Configure RX Timer for first edge interrupt */
    LL_TIM_IC_SetPolarity(RX_TIM(PortNum), LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_ClearFlag_CC1(RX_TIM(PortNum)); //AR to avoid 2 interrupt RX timer callback (with a reset of the decfields vars
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;

    if (Ports[PortNum].decfields.exed_flag != 1)
    {
      /* Get the number of bit received*/
      bitsize = Ports[PortNum].decfields.k * 32 + Ports[PortNum].decfields.j;

      /* Evaluate callback*/
      if ((Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed != NULL) && (bitsize > 0))
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
#endif
}

void USBPDM1_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  /* Set the correct pin on the comparator*/
  Ports[PortNum].CCx = cc;
  USBPDM1_COMP_SetCC(PortNum, cc);
}

__STATIC_INLINE void USBPDM1_SetRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role)
{
  if (role == USBPD_PORTPOWERROLE_SRC)
  {
    /* SRC Case */
    if (PortNum == 0)
    {
      /* Set Rp */
      USBPDM1_AssertRp(0);
      /* UnSet Rd */
      USBPDM1_GPIO_Off(HRD_P0);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P0);
      USBPDM1_GPIO_On(ENCC1_P0);
    }
#if (USBPD_PORT_COUNT==2)
    if (PortNum == 1)
    {
      /* Set Rp */
      USBPDM1_AssertRp(1);
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
    /* SNK Case*/
    if (PortNum == 0)
    {
      /* Set Rd */
      USBPDM1_GPIO_On(HRD_P0);
      /* Unset Rp */
      USBPDM1_DeAssertRp(0);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P0);
      USBPDM1_GPIO_On(ENCC1_P0);
    }
#if (USBPD_PORT_COUNT==2)
    if (PortNum == 1)
    {
      /* Set Rd */
      USBPDM1_GPIO_On(HRD_P1);
      /* Unset Rp */
      USBPDM1_DeAssertRp(1);
      /* Enable both CC */
      USBPDM1_GPIO_On(ENCC2_P1);
      USBPDM1_GPIO_On(ENCC1_P1);
    }
#endif
  }
}

void USBPDM1_AssertRp(uint8_t PortNum)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Allow to know Resistor config is Rp */
  Ports[PortNum].resistor = 1;

  /* Configure the GPIO pin */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  if (PortNum == 0)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P0].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P0].GPIOx, USBPDM1_GPIOs[HRP_P0].GPIO_Pin, HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P0].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
    (USBPDM1_GPIOs[HRP_P0].GPIOx)->BSRR = (uint32_t)(USBPDM1_GPIOs[HRP_P0].GPIO_Pin);
  }
#if (USBPD_PORT_COUNT==2)
  if (PortNum == 1)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx, USBPDM1_GPIOs[HRP_P1].GPIO_Pin, HRP_POSITION(PortNum));
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
  if (PortNum == 0)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P0].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P0].GPIOx, USBPDM1_GPIOs[HRP_P0].GPIO_Pin, HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P0].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
  }
#if (USBPD_PORT_COUNT==2)
  if (PortNum == 1)
  {
    GPIO_InitStruct.Pin = USBPDM1_GPIOs[HRP_P1].GPIO_Pin;
    SINGLE_GPIO_DeInit(USBPDM1_GPIOs[HRP_P1].GPIOx, USBPDM1_GPIOs[HRP_P1].GPIO_Pin, HRP_POSITION(PortNum));
    SINGLE_GPIO_Init(USBPDM1_GPIOs[HRP_P1].GPIOx, &GPIO_InitStruct, HRP_POSITION(PortNum));
  }
#endif
}

void USBPDM1_AssertRd(uint8_t PortNum)
{
  /* Allow to know Resistor config is Rp */
  Ports[PortNum].resistor = 0;
  if (PortNum == 0)
  {
    USBPDM1_GPIO_On(HRD_P0);
  }
#if (USBPD_PORT_COUNT==2)
  if (PortNum == 1)
  {
    USBPDM1_GPIO_On(HRD_P1);
  }
#endif
}


void USBPDM1_DeAssertRd(uint8_t PortNum)
{
  if (PortNum == 0)
  {
    USBPDM1_GPIO_Off(HRD_P0);
  }
#if (USBPD_PORT_COUNT==2)
  if (PortNum == 1)
  {
    USBPDM1_GPIO_Off(HRD_P1);
  }
#endif
}

void HW_SignalDetachment(uint8_t PortNum, CCxPin_TypeDef cc)
{
  USBPDM1_RX_DisableInterrupt(PortNum);
  USBPDM1_Set_CC(PortNum, CCNONE);
  USBPDM1_ReInitRXD(PortNum, cc);
}

/* -------------------------------------------------------------------------- */
/* ------------------------ OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
  * @brief  De-initialize the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx     where x can be (A..F) to select the GPIO peripheral for STM32F0 family
  * @param  GPIO_Pin  Specify the port bit to be written.
  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  position  Position of the GPIO pin
  * @retval None
  */
__STATIC_INLINE void SINGLE_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin, uint32_t position)
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
  if (tmp == (GPIO_GET_INDEX(GPIOx) << (4 * (position & 0x03))))
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
  * @param  GPIOx       where x can be (A..F) to select the GPIO peripheral for STM32F0 family
  * @param  GPIO_Init   pointer to a GPIO_InitTypeDef structure that contains
  *                     the configuration information for the specified GPIO peripheral.
  * @param  position  Position of the GPIO pin
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
  if ((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
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
  * @param  htim    TIM Output Compare handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  tim_it  TIM Channel IT
  * @retval HAL status
  */
#if defined(USE_HAL_TIM)
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Disable the TIM Capture/Compare 1 interrupt */
  __HAL_TIM_DISABLE_IT(htim, tim_it);

  /* Disable the Output compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);

  if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
  {
    /* Disable the Main Ouput */
    LL_TIM_DisableAllOutputs(htim->Instance);/*->BDTR &= ~(TIM_BDTR_MOE);*/
  }
  /* Disable the Peripheral */
  LL_TIM_DisableCounter(htim->Instance);
}
#else
__STATIC_INLINE void SINGLE_TIM_OC_Stop_IT(uint8_t PortNum, uint32_t Channel)
{
  LL_TIM_DisableIT_CC1(RX_COUNTTIM(PortNum));
  LL_TIM_CC_DisableChannel(RX_COUNTTIM(PortNum), Channel);
  if (IS_TIM_BREAK_INSTANCE(RX_COUNTTIM(PortNum)) != RESET)
  {
    /* Disable the Main Ouput */
    LL_TIM_DisableAllOutputs(RX_COUNTTIM(PortNum));
  }
  LL_TIM_DisableCounter(RX_COUNTTIM(PortNum));
}
#endif /* USE_HAL_TIM */

#if defined(USE_HAL_SPI)
__STATIC_INLINE void SINGLE_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  /* Enable the Tx DMA channel */
  HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)pData, (uint32_t)&hspi->Instance->DR, Size);
  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(hspi);
  /* Enable Tx DMA Request */
  LL_SPI_EnableDMAReq_TX(hspi->Instance);
}
#endif /* USE_HAL_SPI */

/**
  * @brief  Stops the TIM Input Capture measurement in interrupt mode.
  * @param  htim    TIM handle
  * @param  Channel TIM Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  tim_it  TIM Channel IT
  * @retval HAL status
  */
#if defined(USE_HAL_TIM)
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
#else
__STATIC_INLINE void SINGLE_TIM_IC_Stop_IT(uint8_t PortNum, uint32_t Channel)
{
  LL_TIM_DisableIT_CC1(RX_TIM(PortNum));
  LL_TIM_CC_DisableChannel(RX_TIM(PortNum), Channel);
  LL_TIM_DisableCounter(RX_TIM(PortNum));
}
#endif /* USE_HAL_TIM */

#if defined(USE_HAL_TIM)
void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it)
{
  SINGLE_TIM_IC_Stop_IT(htim, Channel, tim_it);
}
#else
void USBPD_SINGLE_TIM_IC_Stop_IT(uint8_t PortNum, uint32_t Channel)
{
  SINGLE_TIM_IC_Stop_IT(PortNum, Channel);
}
#endif /* USE_HAL_TIM */

void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HardResetMode_TypeDef Mode)
{
  /* Reset the BIST index*/
  Ports[PortNum].BIST_index = 0;
  /* Reset the Bypass Bus Idle Flag */
  Ports[PortNum].BusIdleFlg = 0;
  /* Initialize State */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  RX_Init_Hvar(PortNum);
}

/* -------------------------------------------------------------------------- */
/* ----------------- END OF OPTIMIZED FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

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

