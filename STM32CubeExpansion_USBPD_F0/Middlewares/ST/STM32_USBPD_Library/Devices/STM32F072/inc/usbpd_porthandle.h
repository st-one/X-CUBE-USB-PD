/**
  ******************************************************************************
  * @file    usbpd_porthandle.h
  * @author  MCD Application Team
  * @version $VERSION$
  * @date    $DATE$
  * @brief   This file contains the headers of usbpd_porthandle.h.
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

#ifndef __USBPD_PORTHANDLE_H_
#define __USBPD_PORTHANDLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_bus.h"
#if defined(USE_HAL_TIM)
#else
#include "stm32f0xx_ll_tim.h"
#endif /* USE_HAL_TIM */

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PORTHANDLE
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/**
  * @brief Struct used for the decodig phase at HW_IF level
  *
  */
typedef struct
{
  uint32_t     DMA_count;          /**<Actual DMA Count value */
  uint32_t     curr_indx;         /**<This represents the position index to append the new coded data */
  uint8_t      preamble;           /**<Flag to identify if preamble ended (Ended = 1 - Otherwise 0 ) */
  uint32_t     k;                  /**<word index on RXBuf*/
  uint32_t     j;                  /**<bit index on RXBuf*/
  uint8_t      prev_bit;
  uint8_t      exed_flag;          /**<bit to prevent overflow*/
  uint32_t     temp_data;          /**<temporary data received*/
}
FiveBitCodingVar_TypeDef;

/**
  * @brief Enum used to get the status of decoding
  *
  */
typedef enum
{
  USBPD_PHY_RX_STATUS_NONE,
  USBPD_PHY_RX_STATUS_OK,
  USBPD_PHY_RX_STATUS_SOP_DETECTING,
  USBPD_PHY_RX_STATUS_DATA,
  USBPD_PHY_RX_STATUS_MESSAGE_READY,
  USBPD_PHY_RX_STATUS_ERROR,
  USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP,
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP,
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL,
  USBPD_PHY_RX_STATUS_ERROR_EOP_NOT_FOUND,
  USBPD_PHY_RX_STATUS_ERROR_CRC_FAILED,
}
USBPD_PHY_RX_Status_TypeDef;

/**
  * @brief CallBacks exposed by the HW_IF to the PHY
  *
  */
typedef struct
{
  /**
    * @brief  The message transfer has been completed
    * @param  PortNum The current port number
    * @retval None
    */
  void (*USBPD_HW_IF_TxCompleted)(uint8_t PortNum);
  /**
    * @brief  Bist data sent callback from PHY_HW_IF
    * @param  PortNum Index of current used port
    * @param  bistmode Bist mode
    * @retval None
    */
  void (*USBPD_HW_IF_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);
  /**
    * @brief  A new message is incoming, need to reset the status.
    * @param  PortNum The current port number
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef(*USBPD_HW_IF_RX_Reset)(uint8_t PortNum);
  /**
  * @brief  Some data are avaiable for current message, performe a decoding step.
  * @param  PortNum The current port number
  * @param  data: The last data received
  * @retval The status of the decoding process
  */
  USBPD_PHY_RX_Status_TypeDef(*USBPD_HW_IF_RX_Accumulate)(uint8_t PortNum, uint32_t data);
  /**
  * @brief  The reception phase of the current message is completed, now to complete the decoding, check the message(CRC) and notify it.
  * @param  PortNum The current port number
  * @retval The status of the decoding process
  */
  USBPD_PHY_RX_Status_TypeDef(*USBPD_HW_IF_RX_Completed)(uint8_t PortNum);
} USBPD_HW_IF_Callbacks;

/**
  * @brief HAL_USBPD_PORT State structures definition
  */
typedef enum
{
  HAL_USBPD_PORT_STATE_RESET             = 0x00,    /**< Peripheral is not initialized                      */
  HAL_USBPD_PORT_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use           */
  HAL_USBPD_PORT_STATE_BUSY              = 0x02,    /**< an internal process is ongoing                     */
  HAL_USBPD_PORT_STATE_BUSY_TX           = 0x03,    /**< Data Transmission process is ongoing               */
  HAL_USBPD_PORT_STATE_BUSY_RX           = 0x04,    /**< Data Reception process is ongoing                  */
  HAL_USBPD_PORT_STATE_WAITING           = 0x05,    /**< Waiting for Data Reception process                 */
  HAL_USBPD_PORT_STATE_TIMEOUT           = 0x06,    /**< Timeout state                                      */
  HAL_USBPD_PORT_STATE_ERROR             = 0x07,    /**< Error                                              */
  HAL_USBPD_PORT_STATE_BIST              = 0x08     /**< BIST is on transmission                            */
} HAL_USBPD_PORT_StateTypeDef;

/** @defgroup USBPD_PORT_HandleTypeDef USB PD handle Structure definition for USBPD_PHY_HW_IF
  * @brief  USBPD PORT handle Structure definition
  * @{
  */
typedef struct
{
  USBPD_ParamsTypeDef          *params;
  uint8_t                      Instance;        /*!< USBPD_PORT number                  */
  uint8_t                      *pTxBuffPtr;     /*!< Pointer to Tx Buffer               */
  uint16_t                     TxXferSize;      /*!< Tx Transfer size                   */
  uint8_t                      *pRxBuffPtr;     /*!< Pointer to Raw Rx transfer Buffer  */
  uint32_t                     *pRxDataPtr;     /*!< Pointer to 5bdecoded data          */
  uint16_t                     RxXferSize;      /*!< Rx Transfer size                   */
  CCxPin_TypeDef               CCx;             /*!< CC pin used for communication      */
#if defined(USE_HAL_SPI)
  SPI_HandleTypeDef            hspitx;          /*!< Tx SPI Handle parameters           */
#endif /* USE_HAL_SPI */
#if defined(USE_HAL_TIM)
  TIM_HandleTypeDef            htimtx;          /*!< Tx TIM Handle parameters           */
#endif /* USE_HAL_TIM */
#if defined(USE_HAL_SPI)
  DMA_HandleTypeDef            hdmatx;          /*!< Tx DMA Handle parameters           */
#endif /* USE_HAL_SPI */
  COMP_HandleTypeDef           hcomprx;         /*!< Rx Comparator Handle parameters    */
#if defined(USE_HAL_TIM)
  TIM_HandleTypeDef            htimrx;          /*!< Rx TIM Handle parameters           */
  TIM_HandleTypeDef            htimcountrx;     /*!< Rx COUNTER TIM Handle parameters   */
#endif /* USE_HAL_TIM */
  DMA_HandleTypeDef            hdmarx;          /*!< Rx DMA Handle parameters           */
  HAL_LockTypeDef              Lock;            /*!< Locking object                     */
  HAL_USBPD_PORT_StateTypeDef  State;           /*!< Communication state                */
  __IO uint32_t                ErrorCode;       /*!< Error code                         */
  FiveBitCodingVar_TypeDef     decfields;       /*!< Fields used for decoding           */
  USBPD_HW_IF_Callbacks        cbs;             /*!< USBPD_PHY_HW_IF callbacks         */
  uint8_t                      resistor;        /*!< The resistor 1 if RP asserted else 0 */
  uint32_t                     BIST_index;      /*!< Index for monitoring BIST Msg bits */
  __IO uint8_t                 BusIdleFlg;      /*!< Disable the check bus idle func on tx */
} USBPD_PORT_HandleTypeDef;
/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
#define USBPD_LOWEST_IRQ_PRIO   3   /*!< Lowest priority           */
#define USBPD_LOW_IRQ_PRIO      1   /*!< High priority shift value */
#define USBPD_HIGH_IRQ_PRIO     0   /*!< Low priority shift value  */
#define RX_IRQ_PRIO             0   /*!< Rx priority for first interrupt */
#define TX_IRQ_PRIO             0   /*!< Communication is half duplex so tx prio = rx prio */

/* IRQ priorities for each port
                  PORT0     PORT1
  ADC               3
  RX_TIM            0         1
  TX_DMA_TC         0         1
  RX_COUNT_TIM      2         3

*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#if !defined(USE_HAL_SPI)
/* Definition for SPI to use LL driver instead of HAL driver */
#define LL_SPI_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2); \
                              else \
                                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1); \
                              } while(0)

#define LL_SPI_GPIO_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); \
                              else \
                                LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); \
                              } while(0)

#define LL_SPI_TX_CLK_PIN(__PORT__)            ((__PORT__ == 0) ? LL_GPIO_PIN_13 :  LL_GPIO_PIN_3 )
#define LL_SPI_TX_CLK_PORT(__PORT__)           ((__PORT__ == 0) ? GPIOB :  GPIOB )
#define LL_SPI_TX_CLK_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? LL_GPIO_AF_0 :  LL_GPIO_AF_0 )

#define LL_SPI_TX_CC1_PIN(__PORT__)            ((__PORT__ == 0) ? LL_GPIO_PIN_14 :  LL_GPIO_PIN_4 )
#define LL_SPI_TX_CC1_PORT(__PORT__)           ((__PORT__ == 0) ? GPIOB :  GPIOB )
#define LL_SPI_TX_CC1_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? LL_GPIO_AF_0 :  LL_GPIO_AF_0 )

#define LL_SPI_TX_CC2_PIN(__PORT__)            ((__PORT__ == 0) ? LL_GPIO_PIN_2 :  LL_GPIO_PIN_6 )
#define LL_SPI_TX_CC2_PORT(__PORT__)           ((__PORT__ == 0) ? GPIOC :  GPIOA )
#define LL_SPI_TX_CC2_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? LL_GPIO_AF_1 :  LL_GPIO_AF_0 )


#define LL_DMACH_GET_FLAG_TC(__PORT__, __DMA__)       ((__PORT__ == 0) ? LL_DMA_IsActiveFlag_TC5(__DMA__) :  LL_DMA_IsActiveFlag_TC3(__DMA__ ))
#define LL_DMACH_CLEAR_FLAG_GI(__PORT__, __DMA__)     ((__PORT__ == 0) ? LL_DMA_ClearFlag_GI5(__DMA__) :  LL_DMA_ClearFlag_GI3(__DMA__ ))
#endif /* USE_HAL_SPI */

#define TX_SPI(__PORT__)                ((__PORT__ == 0) ? SPI2 :     SPI1 )
#define TX_SCK_GPIOPORT(__PORT__)       ( GPIOB )
#define TX_SCK_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_13 :  GPIO_PIN_3 )
#define TX_CC1_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF0_SPI2 :  GPIO_AF0_SPI1 )
#define TX_CC2_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF1_SPI2 :  GPIO_AF0_SPI1 )
#define TX_CLK_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF0_SPI2 :  GPIO_AF0_SPI1 )
#define TX_CC1_GPIOPORT(__PORT__)  ( GPIOB )
#define TX_CC1_PIN(__PORT__)    ((__PORT__ == 0) ? GPIO_PIN_14 :  GPIO_PIN_4 )
#define TX_CC1_PIN_POSITION(__PORT__)  ((__PORT__ == 0) ? 14 :                  4 )
#define TX_CC2_GPIOPORT(__PORT__)  ((__PORT__ == 0) ? GPIOC :      GPIOA )
#define TX_CC2_PIN(__PORT__)    ((__PORT__ == 0) ? GPIO_PIN_2 :    GPIO_PIN_6 )
#define TX_CC2_PIN_POSITION(__PORT__)  ((__PORT__ == 0) ? 2 :                  6 )
#define TX_DMA(__PORT__)     ((__PORT__ == 0) ? DMA1 :   DMA1 )
#define TX_DMACH(__PORT__)     ((__PORT__ == 0) ? DMA1_Channel5 :   DMA1_Channel3 )
#define TX_DMACH_NUMBER(__PORT__)     ((__PORT__ == 0) ? LL_DMA_CHANNEL_5 :   LL_DMA_CHANNEL_3 )
#define TX_DMACHIRQ(__PORT__)     ((__PORT__ == 0) ? DMA1_Channel4_5_6_7_IRQn :   DMA1_Channel2_3_IRQn )
#define TX_DMACHIRQ_PRIO(__PORT__)   ((__PORT__ == 0) ? (TX_IRQ_PRIO + 2 + USBPD_HIGH_IRQ_PRIO) : (TX_IRQ_PRIO + 2 + USBPD_HIGH_IRQ_PRIO) )
#define TX_TIM(__PORT__)      ((__PORT__ == 0) ? TIM14 :       TIM15 )
#if defined(USE_HAL_TIM)
#define TX_TIMCH(__PORT__)           ((__PORT__ == 0) ? TIM_CHANNEL_1 :   TIM_CHANNEL_2 )
#define TX_TIM_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_1 :    GPIO_PIN_15 )
#define TX_TIM_GPIOAF(__PORT__)         ((__PORT__ == 0) ? GPIO_AF0_TIM14 :  GPIO_AF1_TIM15 )
#else
#define TX_TIMCH(__PORT__)           ((__PORT__ == 0) ? LL_TIM_CHANNEL_CH1 :   LL_TIM_CHANNEL_CH2 )
#define TX_TIM_PIN(__PORT__)            ((__PORT__ == 0) ? LL_GPIO_PIN_1 :    LL_GPIO_PIN_15 )
#define TX_TIM_GPIOAF(__PORT__)         ((__PORT__ == 0) ? LL_GPIO_AF_0 :  LL_GPIO_AF_1 )
#endif /* USE_HAL_TIM */
#define TX_TIM_GPIOPORT(__PORT__)       ( GPIOB )

#define RX_TIM(__PORT__)      ((__PORT__ == 0) ? TIM3 :       TIM1 )
#if defined(USE_HAL_TIM)
#define RX_TIMCH(__PORT__)     ( TIM_CHANNEL_1 )
#define RX_TIMCH_TIMIT(__PORT__)        ( TIM_IT_CC1 )
#define RX_TIM_DMA_CC(__PORT__)         ( TIM_DMA_CC1 )
#define RX_TIM_DMA_ID_CC(__PORT__)         ( TIM_DMA_ID_CC1 )
#else
#define RX_TIMCH(__PORT__)     ( LL_TIM_CHANNEL_CH1 )
#endif /* USE_HAL_TIM */
#define RX_TIM_IRQN(__PORT__)     ((__PORT__ == 0) ? TIM3_IRQn : TIM1_CC_IRQn )
#define RX_TIM_IRQ_PRIO(__PORT__)   ((__PORT__ == 0) ? (RX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO) : (RX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO) )
#define RX_COMP(__PORT__)    ((__PORT__ == 0) ? COMP1 : COMP2 )
#define RX_COMPOUT(__PORT__)    ((__PORT__ == 0) ? COMP_OUTPUT_TIM3IC1 : COMP_OUTPUT_TIM1IC1 )
#define RX_DMACH(__PORT__)     ((__PORT__ == 0) ? DMA1_Channel4 :   DMA1_Channel2 )
#define RX_COUNTTIM(__PORT__)      ((__PORT__ == 0) ? TIM16 :       TIM17 )
#define RX_COUNTTIM_IRQN(__PORT__)   ((__PORT__ == 0) ? TIM16_IRQn : TIM17_IRQn )
#define RX_COUNTTIMIRQ_PRIO(__PORT__)   ((__PORT__ == 0) ? (RX_IRQ_PRIO + 1 + USBPD_HIGH_IRQ_PRIO) : (RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO) )
#if defined(USE_HAL_TIM)
#define RX_COUNTTIMCH(__PORT__)         ( TIM_CHANNEL_1 )
#define RX_COUNTTIMCH_TIMIT(__PORT__)   ( TIM_IT_CC1 )
#define RX_COUNTTIMCH_ITFLAG(__PORT__)  ( TIM_FLAG_CC1 )
#else
#define RX_COUNTTIMCH(__PORT__)         ( LL_TIM_CHANNEL_CH1 )
#endif /* USE_HAL_TIM */
#define RX_CC1_GPIOPORT(__PORT__)  ( GPIOA )
#define RX_CC1_PIN(__PORT__)    ((__PORT__ == 0) ? GPIO_PIN_0 :    GPIO_PIN_2 )
#define RX_CC1_PIN_POSITION(__PORT__)  ((__PORT__ == 0) ? 0 :            2 )
#if defined(USE_HAL_ADC)
#define RX_CC1_ADCCH(__PORT__)    ((__PORT__ == 0) ? ADC_CHANNEL_0 :  ADC_CHANNEL_4 )
#else
#define RX_CC1_ADCCH(__PORT__)    ((__PORT__ == 0) ? LL_ADC_CHANNEL_0 :  LL_ADC_CHANNEL_4 )
#endif /* USE_HAL_ADC */
#define RX_CC1_COMPCH(__PORT__)    ( COMP_INVERTINGINPUT_IO1 )
#define RX_CC2_GPIOPORT(__PORT__)  ( GPIOA )
#define RX_CC2_PIN(__PORT__)    ((__PORT__ == 0) ? GPIO_PIN_5 :    GPIO_PIN_4 )
#define RX_CC2_PIN_POSITION(__PORT__)  ((__PORT__ == 0) ? 5 :            4 )
#if defined(USE_HAL_ADC)
#define RX_CC2_ADCCH(__PORT__)    ((__PORT__ == 0) ? ADC_CHANNEL_2 :  ADC_CHANNEL_5 )
#else
#define RX_CC2_ADCCH(__PORT__)    ((__PORT__ == 0) ? LL_ADC_CHANNEL_2 :  LL_ADC_CHANNEL_5 )
#endif /* USE_HAL_ADC */
#define RX_CC2_COMPCH(__PORT__)    ((__PORT__ == 0) ? COMP_INVERTINGINPUT_DAC2 : COMP_INVERTINGINPUT_DAC1 )
#define HRP_POSITION(__PORT__)          ((__PORT__ == 0) ? 12 :            7 )

#define RX_REF_GPIOPORT(__PORT__)       ( GPIOA )
#define RX_REF_PIN(__PORT)              ( GPIO_PIN_1 )
#if defined(USE_HAL_ADC)
#define RX_REF_ADCCH(__PORT__)          ( ADC_CHANNEL_1 )
#else
#define RX_REF_ADCCH(__PORT__)          ( LL_ADC_CHANNEL_1 )
#endif /* USE_HAL_ADC */

#define SPI_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_SPI2_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_SPI1_CLK_ENABLE(); \
                              } while(0)

#define IS_TX_TIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == TX_TIM(1)) ) )
#define IS_RX_TIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_TIM(0)) || (htim_base->Instance == RX_TIM(1)) ) )
#define IS_RX_COUNTTIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_COUNTTIM(0)) || (htim_base->Instance == RX_COUNTTIM(1)) ) )

#define GET_PORT_FROM_TIM(htim_base) \
( (uint8_t)( (htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == RX_TIM(0)) || (htim_base->Instance == RX_COUNTTIM(0)) )? 0 : 1 )

#if defined(USE_HAL_TIM)
#define TX_TIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM14_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM15_CLK_ENABLE(); \
                              } while(0)

#define RX_TIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM3_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM1_CLK_ENABLE(); \
                              } while(0)

#define RX_COUNTTIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM16_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_ENABLE(); \
                              } while(0)

#define TX_TIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM14_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM15_CLK_DISABLE(); \
                              } while(0)

#define RX_TIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM3_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM1_CLK_DISABLE(); \
                              } while(0)

#define RX_COUNTTIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                __HAL_RCC_TIM16_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_DISABLE(); \
                              } while(0)
#else
#define TX_TIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14); \
                              else \
                                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM15); \
                              } while(0)
                                
#define RX_TIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); \
                              else \
                                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1); \
                              } while(0)

#define RX_COUNTTIM_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16); \
                              else \
                                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17); \
                              } while(0)

#define RX_TIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3); \
                              else \
                                LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_TIM1); \
                              } while(0)

#define TX_TIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM14); \
                              else \
                                LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_TIM15); \
                              } while(0)

#define RX_COUNTTIM_CLK_DISABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_TIM16); \
                              else \
                                LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_TIM17); \
                              } while(0)
#endif /* USE_HAL_TIM */
                                


/* MSP MACRO */
                                
#if defined(USE_HAL_TIM)
#else
#define TX_TIMCH_GPIO_CLK_ENABLE(PortNum) do { \
                              if (PortNum==0) \
                                LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); \
                              else \
                                LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); \
                              } while(0);

#define TX_TIM_SET_COMPARE(PortNum, CompareValue) do { \
                              if (PortNum==0) \
                                LL_TIM_OC_SetCompareCH1(TX_TIM(PortNum), CompareValue); \
                              else \
                                LL_TIM_OC_SetCompareCH2(TX_TIM(PortNum), CompareValue); \
                              } while(0)

#define RX_COUNTTIM_SET_COMPARE(PortNum, CompareValue)  do { \
                              if (PortNum==0) \
                                LL_TIM_OC_SetCompareCH1(RX_COUNTTIM(PortNum), CompareValue); \
                              else \
                                LL_TIM_OC_SetCompareCH1(RX_COUNTTIM(PortNum), CompareValue); \
                              } while(0);
#endif /* USE_HAL_TIM */

#define GET_PORT_FROM_SPI(hspi) \
( (uint8_t)( hspi->Instance == TX_SPI(0) )? 0 : 1 )

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PORTHANDLE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
