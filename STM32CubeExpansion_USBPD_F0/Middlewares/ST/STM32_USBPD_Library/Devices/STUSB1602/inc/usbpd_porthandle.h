/**
  ******************************************************************************
  * @file    usbpd_porthandle.h
  * @author  System Lab
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
#include "STUSB1602_Driver.h"
#include "usbpd_def.h"
  
  /**
  * @addtogroup USBPD_PHY
  * @{
  * */


/**
  * @brief Default current limit of the power switches supplying VCONN on the CC pins  
  */
typedef enum 
{
  VConn_Ilim_350mA = 0x00,
  VConn_Ilim_300mA = 0x01,
  VConn_Ilim_250mA = 0x02,
  VConn_Ilim_200mA = 0x03,
  VConn_Ilim_150mA = 0x04,
  VConn_Ilim_100mA = 0x05,
  VConn_Ilim_400mA = 0x06,
  VConn_Ilim_450mA = 0x07,
  VConn_Ilim_500mA = 0x08,
  VConn_Ilim_550mA = 0x09,
  VConn_Ilim_600mA = 0x0A
} USBPD_VConnIlim_TypeDef;

  
  /* Exported typedef ----------------------------------------------------------*/
  
typedef enum 
{
  STUSB16xx_SPI_Mode_TX = 0,
  STUSB16xx_SPI_Mode_RX = 1
} STUSB1602_SPI_Mode_TypeDef;


  /**
  * @brief Struct used for the decodig phase at HW_IF level
  * */
  typedef struct
  {   
    uint8_t     exed_flag;              /* Exit cases */
    uint8_t     preamble;               /**<Flag to identify if preamble ended (Ended = 1 - Otherwise 0 ) */
    uint32_t    dataindex;              /* index of RxDataBuf */
    uint32_t    dataoffset;             /* offset of RxDataBuf */
    uint32_t    index;                  /* word index of RXBuf */
    uint32_t    offset;                 /* bit index of RXBuf */
  } UnwrapData_TypeDef;

  /**
  * @brief Enum used to get the status of decoding
  * */
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
  * */
  typedef struct
  {
    /**
      * @brief  A message has been received on a specified port.
      * @param  hport: The current port number
      * @param  pPayload: pointer to the received data buffer in 5bit representation
      * @param  Bitsize: number of bit received 
      * @retval None
      */ 
    USBPD_PHY_RX_StatusTypeDef (*USBPD_HW_IF_ReceiveMessage)(uint8_t hport, uint32_t *pPayload, uint16_t Bitsize); 
    /**
      * @brief  The message transfer has been completed
      * @param  hport: The current port number
      * @retval None
      */ 
    void (*USBPD_HW_IF_TxCompleted)(uint8_t hport);
    /**
      * @brief  Bist data sent callback from PHY_HW_IF
      * @param  hport: Index of current used port
      * @param  bistmode: Bist mode
      * @retval None
      */
    void (*USBPD_HW_IF_BistCompleted)(uint8_t hport, USBPD_BISTMsg_TypeDef bistmode);
    /**
    * @brief  A new message is incoming, need to reset the status.
    * @param  hport: The current port number
    * @retval The status of the decoding process
    */ 
    USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Reset)(uint8_t hport);
    /**
    * @brief  Some data are avaiable for current message, performe a decoding step.
    * @param  hport: The current port number
    * @param  data: The last data received
    * @retval The status of the decoding process
    */ 
    USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Accumulate)(uint8_t hport, uint32_t data);
    /**
    * @brief  The reception phase of the current message is completed, now to complete the decoding, check the message(CRC) and notify it.
    * @param  hport: The current port number
    * @retval The status of the decoding process
    */ 
    USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Completed)(uint8_t hport);       
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
  }HAL_USBPD_PORT_StateTypeDef;

  
/** @defgroup USBPD_PORT_HandleTypeDef USB PD handle Structure definition for @ref USBPD_HW_IF
  * @brief  USBPD PORT handle Structure definition
  * @{
  */
  typedef struct
  {
    uint8_t		        Instance;        /*!< USBPD_PORT number                  */
    uint8_t                     *pTxBuffPtr;     /*!< Pointer to Tx Buffer               */
    uint16_t                    TxXferSize;      /*!< Tx Transfer size                   */
    uint8_t                     *pRxBuffPtr;     /*!< Pointer to Raw Rx transfer Buffer  */
    uint32_t                    *pRxDataPtr;     /*!< Pointer to 5bdecoded data          */
    uint16_t                    RxXferSize;      /*!< Rx Transfer size                   */
    CCxPin_TypeDef              CCx;             /*!< CC pin used for communication      */ 
    FlagStatus                  CCxChange;       /*!< CC event change                    */ 
    HAL_LockTypeDef             Lock;            /*!< Locking object                     */
    HAL_USBPD_PORT_StateTypeDef State;           /*!< Communication state                */
    __IO uint32_t               ErrorCode;       /*!< Error code                         */
    USBPD_PortPowerRole_TypeDef role;            /*!< The Role of the port Provider or Consumer */
    uint32_t                    BIST_index;      /*!< Index for monitoring BIST Msg bits */
    FunctionalState             VConn;           /*!< VConn status flag                  */
    USBPD_PortDataRole_TypeDef  DataRole;        /*!< Data role of the port              */
    uint8_t                     TxSpareBits;   /*!< TxSpareBits */ 
    UnwrapData_TypeDef          unwrapdata;      /*!< Fields used for decoding           */
    SPI_HandleTypeDef           hspi;            /*!< SPI Handle parameters              */
    DMA_HandleTypeDef           hdmatx;          /*!< Tx DMA Handle parameters           */
    DMA_HandleTypeDef           hdmarx;          /*!< Rx DMA Handle parameters           */
    I2C_HandleTypeDef           hi2c;            /*!< I2C Handle parameters              */
    TIM_HandleTypeDef           htimcountrx;     /*!< Rx COUNTER TIM Handle parameters   */
    USBPD_HW_IF_Callbacks       cbs;             /*!< @ref USBPD_HW_IF callbacks         */
    uint8_t                     AlertEventCount;
    uint8_t                     CommLock;
    DEVICE_CUT_TypeDef          Device_cut;
  }STUSB16xx_PORT_HandleTypeDef;
  /** 
  * @}
  */

/** @defgroup USBPD_Init_TypeDef USB PD Initialization Structure definition to be moved in usbpd_def.h
  * @brief  USBPD Init Structure definition
  * @{
  */
  typedef struct
  {
    uint8_t		                        Instance;                 /* USBPD_PORT number */
    USBPD_PortPowerRole_TypeDef                 RolePower;                /* It defines the power role */
    USBPD_PortDataRole_TypeDef                  RoleData;                 /* It defines the initial data role */
    FunctionalState                             VendorMessages;           /* If enabled, It allows sending vendor messages */
    FunctionalState                             Ping;                     /* If enabled it allows sending Ping message when an Explicit Contract is established */
    FunctionalState                             ExtendedMessages;         /* If enabled it supports extended messages */
    uint16_t                                    PE_SCAP_HR;               /* Number of source capabilities requests before hard reset */
    Current_Capability_Advertised_TypeDef       CCCurrentAdvertised;      /* It advertises the current capability */
    FunctionalState                             DataRoleSwap;             /* It enables or disables the data role swap capability */
    FunctionalState                             PowerRoleSwap;            /* It enables or disables the power role swap capability */
    FunctionalState                             VConnSwap;                /* It enables or disables the VCONN swap capability */
    FunctionalState                             VConnSupply;              /* It enables or disables the VCONN supply capability on CC pin */
    FunctionalState                             VConnDischarge;           /* It enables or disables the VCONN discharge on CC pin */
    FunctionalState                             VBusDischarge;            /* It enables or disables the Vbus discharge */
    USBPD_VConnIlim_TypeDef                     VConnIlim;                /* It allows changing the default current limit supplying VCONN on the CC pins */
    FunctionalState                             VConnMonitoring;          /* It enables or disables UVLO threshold detection on VCONN pin */
    VCONN_UVLO_Threshold_TypeDef                VConnThresholdUVLO;       /* High UVLO threshold of 4.65 V; 1b: Low UVLO threshold of 2.65 V (case of VCONN-powered accessories operating down to 2.7 V) */
    uint16_t                                    VBusSelect;               /* Value (mV) related to targeted VBUS voltage used for VBUS range monitoring by 100mV step */
    uint8_t                                     VbusVShiftHigh;           /* Shift coefficient used for computing the high threshold value (5% + VBUS_VSHIFT_HIGH) of the monitoring voltage range */
    uint8_t                                     VbusVShiftLow;            /* Shift coefficient used for computing the low threshold value (-5% - VBUS_VSHIFT_LOW) of the monitoring voltage range */
    FunctionalState                             VBusRange;                /* It enables or disables VBUS voltage range detection */
    VBUS_VSAFE0V_Threshold_TypeDef              VBusThresholdVSafe0V;     /* VBus vSafe0V threshold */
    FunctionalState                             VddOVLO;                  /* It enables or disables OVLO threshold detection on Vdd voltage */
    FunctionalState                             VddUVLO;                  /* It enables or disables UVLO threshold detection on Vdd voltage */
    uint8_t                                     VBusDischargeTimeTo0V;    /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to 0 V: 84 ms*TDISPARAM (840ms is default discharge time) */
    uint8_t                                     VBusDischargeTimeToPDO;   /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to PDO: 20 ms*TDISPARAM (200 ms is default discharge time) */
    FunctionalState                             PowerAccessoryDetection;  /* It enables or disables powered accessory detection */
    FunctionalState                             PowerAccessoryTransition; /* It enables or disables powered accessory transition from Powered.Accessory state to Try.SNK */
  }USBPD_Init_TypeDef;
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
  
  
#ifdef MB1303
 
                                     
                                
  /* Exported variables --------------------------------------------------------*/
  /* Exported functions --------------------------------------------------------*/
  


/* STUSB1602 IOs */
#define TX_EN_GPIO_PORT(__PORT__)	        ( GPIOC )
#define TX_EN_GPIO_PIN(__PORT__)	        ((__PORT__ == 0) ? GPIO_PIN_2   : GPIO_PIN_3 )  
                                
#define RESET_GPIO_PORT(__PORT__)	        ( GPIOC )
#define RESET_GPIO_PIN(__PORT__)	        ((__PORT__ == 0) ? GPIO_PIN_6   : GPIO_PIN_7 )   

#define A_B_Side_GPIO_PORT(__PORT__)            ( GPIOA )
#define A_B_Side_GPIO_PIN(__PORT__)             ((__PORT__ == 0) ? GPIO_PIN_0   : GPIO_PIN_3 )
                                
#define ALERT_PORT_INDEX(__PORT__)              ((__PORT__ == 0) ? 0            : 1 )  
#define ALERT_GPIO_PORT(__PORT__)               ( GPIOA )  
#define ALERT_GPIO_PIN(__PORT__)                ((__PORT__ == 0) ? GPIO_PIN_1   : GPIO_PIN_2 )
#define ALERT_GPIO_IRQHANDLER(__PORT__)         ((__PORT__ == 0) ? EXTI0_1_IRQn : EXTI2_3_IRQn )
#define ALERT_GPIO_IRQPRIORITY(__PORT__)        ( USBPD_LOW_IRQ_PRIO )
  
/* I2C */
#define I2C_INSTANCE(__PORT__)                  ( I2C2 )
#define I2C_TIMING(__PORT__)                    ( 0x20100917 ) //0x20303E5D --> 100kHz  // 0x2010091A; //--> 400kHz;
  
#define I2C_PORT(__PORT__)                      ( GPIOB )
#define I2C_SCL_PIN(__PORT__)                   ( GPIO_PIN_10 )
#define I2C_SCL_PORT(__PORT__)                  ( GPIOB )
#define I2C_SDA_PIN(__PORT__)                   ( GPIO_PIN_11 )
#define I2C_SDA_PORT(__PORT__)                  ( GPIOB )
#define I2C_MODE(__PORT__)                      ( GPIO_MODE_AF_OD )
#define I2C_PULL(__PORT__)                      ( GPIO_NOPULL )
#define I2C_SPEED(__PORT__)                     ( GPIO_SPEED_FREQ_HIGH )  
#define I2C_ALTERNATE(__PORT__)                 ( GPIO_AF1_I2C2 )  
   
    
/* SPI */ 
#define SPI_Instance(__PORT__)		        ((__PORT__ == 0) ? SPI2 : 		SPI1 ) /* Ok for STUSB16xx_EVAL */

#define SPI_NSS_PORT(__PORT__)                  ((__PORT__ == 0) ? GPIOB                        :       GPIOA )  
#define SPI_NSS_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_12                  :       GPIO_PIN_15 )
#define SPI_NSS_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF0_SPI2                :       GPIO_AF0_SPI1 )
                                
#define SPI_NSS_LL_APB(__PORT__)                ( LL_APB1_GRP2_PERIPH_SYSCFG )
#define SPI_NSS_LL_PORT(__PORT__)               ((__PORT__ == 0) ? LL_SYSCFG_EXTI_PORTB         :       LL_SYSCFG_EXTI_PORTA )
#define SPI_NSS_LL_SYS_EXTI(__PORT__)           ((__PORT__ == 0) ? LL_SYSCFG_EXTI_LINE12        :       LL_SYSCFG_EXTI_LINE15 )
#define SPI_NSS_LL_EXTI(__PORT__)               ((__PORT__ == 0) ? LL_EXTI_LINE_12              :       LL_EXTI_LINE_15 )  

#define SPI_CLK_PORT(__PORT__)                  ( GPIOB )  
#define SPI_CLK_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_13                  :       GPIO_PIN_3 )
#define SPI_CLK_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF0_SPI2                :       GPIO_AF0_SPI1 )
                                
#define SPI_MISO_PORT(__PORT__)                 ( GPIOB )
#define SPI_MISO_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_14                  :       GPIO_PIN_4 )
#define SPI_MISO_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF0_SPI2                :       GPIO_AF0_SPI1 )
                                 
#define SPI_MOSI_PORT(__PORT__)                 ( GPIOB )  
#define SPI_MOSI_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_15                  :       GPIO_PIN_5 )
#define SPI_MOSI_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF0_SPI2                :       GPIO_AF0_SPI1 )                     
                                
#define SPI_NSS_LL_IRQHANDLER(__PORT__)         ( EXTI4_15_IRQn )
#define SPI_NSS_LL_IRQPRIORITY(__PORT__)        ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO )                                
                                
#define TX_DMACH(__PORT__) 		        ((__PORT__ == 0) ? DMA1_Channel5                :       DMA1_Channel3 )  
#define RX_DMACH(__PORT__) 		        ((__PORT__ == 0) ? DMA1_Channel4                :       DMA1_Channel2 ) 
#define DMACHIRQ(__PORT__) 		        ((__PORT__ == 0) ? DMA1_Channel4_5_6_7_IRQn     :       DMA1_Channel2_3_IRQn) 
#define DMACHIRQ_PRIO(__PORT__) 	        ( (TX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO))

/* Timers used during decoding phase */ 
#define RX_COUNTTIM(__PORT__)  		        ((__PORT__ == 0) ? TIM16 : 		TIM17 )
#define RX_COUNTTIM_IRQN(__PORT__) 	        ((__PORT__ == 0) ? TIM16_IRQn :         TIM17_IRQn )
#define RX_COUNTTIMIRQ_PRIO(__PORT__) 	        ((__PORT__ == 0) ? (RX_IRQ_PRIO + 0 + USBPD_LOW_IRQ_PRIO) : (RX_IRQ_PRIO + 0 + USBPD_LOW_IRQ_PRIO) )

#define RX_COUNTTIMCH(__PORT__)                 ( TIM_CHANNEL_1 )
#define RX_COUNTTIMCH_TIMIT(__PORT__)           ( TIM_IT_CC1 )
#define RX_COUNTTIMCH_ITFLAG(__PORT__)          ( TIM_FLAG_CC1 )

/* FLR: temporary moved! */
#define DMA_TIME_ELAPSED        60 //us
#define DMA_TIME_COUNT_COMPARE  10 //us

  
/* MSP MACRO */

#define GET_PORT_FROM_I2C(hi2c) \
( (uint8_t)( hi2c->Instance == I2C2)? 0 : 1 )

#define I2C_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_I2C2_CLK_ENABLE(); \
                               else \
                                __HAL_RCC_I2C1_CLK_ENABLE();  \
                              } while(0)

#define I2C_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_I2C2_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_I2C1_CLK_DISABLE();  \
                              } while(0)                                

#define GET_PORT_FROM_SPI(hspi) \
( (uint8_t)( hspi->Instance == SPI_Instance(0) )? 0 : 1 )

#define SPI_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_SPI2_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_SPI1_CLK_ENABLE();  \
                              } while(0)

#define SPI_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_SPI2_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_SPI1_CLK_DISABLE();  \
                              } while(0)                                
                                
                                
#define DMA_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                  __HAL_RCC_DMA1_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_DMA1_CLK_ENABLE();   \
                              } while(0)

                                
#define DMA_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                  __HAL_RCC_DMA1_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_DMA1_CLK_DISABLE();   \
                              } while(0)
                                

                             

                                
#define IS_RX_COUNTTIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_COUNTTIM(0)) || (htim_base->Instance == RX_COUNTTIM(1)) ) )

#define GET_PORT_FROM_TIM(htim_base) \
( (uint8_t)( /*(htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == RX_TIM(0)) ||*/ (htim_base->Instance == RX_COUNTTIM(0)) )? 0 : 1 )




#define RX_COUNTTIM_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM16_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_ENABLE(); \
                              } while(0)

                                
#define RX_COUNTTIM_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM16_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_DISABLE(); \
                              } while(0)  
#else  
  
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/

/* STUSB16xx EVAL IOs */
#define DISCH_PATH_SNK_PORT(__PORT__)           ( GPIOC )  
#define DISCH_PATH_SNK_PIN(__PORT__)            ( GPIO_PIN_14 )
#define DISCH_PATH_SRC_PORT(__PORT__)           ( GPIOC )  
#define DISCH_PATH_SRC_PIN(__PORT__)            ( GPIO_PIN_8) 
#define VBUS_EN_SRC_STM32_PORT(__PORT__)        ( GPIOB)
#define VBUS_EN_SRC_STM32_PIN(__PORT__)         ( GPIO_PIN_0 )
  
/* STUSB1602 IOs */
#define TX_EN_GPIO_PORT(__PORT__)	        ( GPIOB )
#define TX_EN_GPIO_PIN(__PORT__)	        ( GPIO_PIN_14 )  
#define RESET_GPIO_PORT(__PORT__)	        ( GPIOC )
#define RESET_GPIO_PIN(__PORT__)	        ( GPIO_PIN_6 )  

#define A_B_Side_GPIO_PORT(__PORT__)            ( GPIOA )
#define A_B_Side_GPIO_PIN(__PORT__)             ( GPIO_PIN_0 )
  
/* I2C */
#define I2C_INSTANCE(__PORT__)                  ( I2C2 )
#define I2C_TIMING(__PORT__)                    ( 0x20303E5D ) //0x20303E5D --> 100kHz  // 0x20100917; //--> 400kHz;
  
#define I2C_PORT(__PORT__)                      ( GPIOB )
#define I2C_SCL_PIN(__PORT__)                   ( GPIO_PIN_10 )
#define I2C_SCL_PORT(__PORT__)                  ( GPIOB )
#define I2C_SDA_PIN(__PORT__)                   ( GPIO_PIN_11 )
#define I2C_SDA_PORT(__PORT__)                  ( GPIOB )
#define I2C_MODE(__PORT__)                      ( GPIO_MODE_AF_OD )
#define I2C_PULL(__PORT__)                      ( GPIO_NOPULL )
#define I2C_SPEED(__PORT__)                     ( GPIO_SPEED_FREQ_HIGH )  
#define I2C_ALTERNATE(__PORT__)                 ( GPIO_AF1_I2C2 )  

#define ALERT_PORT_INDEX(__PORT__)              ( 0 )  
#define ALERT_GPIO_PORT(__PORT__)               ( GPIOA )  
#define ALERT_GPIO_PIN(__PORT__)                ( GPIO_PIN_5 )
#define ALERT_GPIO_IRQHANDLER(__PORT__)         ( EXTI4_15_IRQn )
#define ALERT_GPIO_IRQPRIORITY(__PORT__)        ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO ) /* USBPD_LOW_IRQ_PRIO */

    
    
/* SPI */ 
#define SPI_Instance(__PORT__)		((__PORT__ == 0) ? SPI2 : 		SPI1 ) /* Ok for STUSB16xx_EVAL */
  
#define SPI_MISO_PORT(__PORT__)                 ( GPIOC )
#define SPI_MISO_PIN(__PORT__)                  ( GPIO_PIN_2 )
#define SPI_MISO_ALTERNATE(__PORT__)            ( GPIO_AF1_SPI2 )
#define SPI_MOSI_PORT(__PORT__)                 ( GPIOC )  
#define SPI_MOSI_PIN(__PORT__)                  ( GPIO_PIN_3 )
#define SPI_MOSI_ALTERNATE(__PORT__)            ( GPIO_AF1_SPI2 )
#define SPI_CLK_PORT(__PORT__)                  ( GPIOB )  
#define SPI_CLK_PIN(__PORT__)                   ( GPIO_PIN_13 )
#define SPI_CLK_ALTERNATE(__PORT__)             ( GPIO_AF0_SPI2 )
#define SPI_NSS_PORT(__PORT__)                  ( GPIOB )  
#define SPI_NSS_PIN(__PORT__)                   ( GPIO_PIN_9 )
#define SPI_NSS_ALTERNATE(__PORT__)             ( GPIO_AF5_SPI2 )
#define SPI_NSS_LL_APB(__PORT__)                ( LL_APB1_GRP2_PERIPH_SYSCFG )
#define SPI_NSS_LL_PORT(__PORT__)               ( LL_SYSCFG_EXTI_PORTB )
#define SPI_NSS_LL_SYS_EXTI(__PORT__)           ( LL_SYSCFG_EXTI_LINE9 )
#define SPI_NSS_LL_EXTI(__PORT__)               ( LL_EXTI_LINE_9 )
#define SPI_NSS_LL_IRQHANDLER(__PORT__)         ( EXTI4_15_IRQn )
#define SPI_NSS_LL_IRQPRIORITY(__PORT__)        ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO )

  
#define TX_DMACH(__PORT__) 		( DMA1_Channel5 )
#define RX_DMACH(__PORT__) 		( DMA1_Channel4 )
#define DMACHIRQ(__PORT__) 		( DMA1_Channel4_5_6_7_IRQn)
#define DMACHIRQ_PRIO(__PORT__) 	( (TX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO))

/* Timers used during decoding phase */ 
#define RX_COUNTTIM(__PORT__)  		((__PORT__ == 0) ? TIM16 : 		TIM17 )
#define RX_COUNTTIM_IRQN(__PORT__) 	((__PORT__ == 0) ? TIM16_IRQn :         TIM17_IRQn )
#define RX_COUNTTIMIRQ_PRIO(__PORT__) 	((__PORT__ == 0) ? (RX_IRQ_PRIO + 0 + USBPD_HIGH_IRQ_PRIO) : (RX_IRQ_PRIO + 0 + USBPD_LOW_IRQ_PRIO) )
#define RX_COUNTTIMCH(__PORT__)         ( TIM_CHANNEL_1 )
#define RX_COUNTTIMCH_TIMIT(__PORT__)   ( TIM_IT_CC1 )
#define RX_COUNTTIMCH_ITFLAG(__PORT__)  ( TIM_FLAG_CC1 )

/* FLR: temporary moved! */
#define DMA_TIME_ELAPSED        60 //us
#define DMA_TIME_COUNT_COMPARE  10 //us

/* Note (GN): To be reviewed */
#define TX_SPI(__PORT__)		((__PORT__ == 0) ? SPI2 : 		SPI1 )
#define TX_SCK_GPIOPORT(__PORT__)	( GPIOB )
#define TX_SCK_PIN(__PORT__)		((__PORT__ == 0) ? GPIO_PIN_13 :	GPIO_PIN_3 )
#define TX_CLK_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF0_SPI2 :	GPIO_AF0_SPI1 )

#define TX_CC1_GPIOPORT(__PORT__)	( GPIOB )
#define TX_CC1_PIN(__PORT__)		((__PORT__ == 0) ? GPIO_PIN_14 :	GPIO_PIN_4 )

#define TX_CC1_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF0_SPI2 :	GPIO_AF0_SPI1 )
#define TX_CC2_SPI_GPIOAF(__PORT__)     ((__PORT__ == 0) ? GPIO_AF1_SPI2 :	GPIO_AF0_SPI1 )

#define TX_CC1_PIN_POSITION(__PORT__)	((__PORT__ == 0) ? 14 :	                4 )
#define TX_CC2_GPIOPORT(__PORT__)	((__PORT__ == 0) ? GPIOC :		GPIOA )
#define TX_CC2_PIN(__PORT__)		((__PORT__ == 0) ? GPIO_PIN_2 :		GPIO_PIN_6 )
#define TX_CC2_PIN_POSITION(__PORT__)	((__PORT__ == 0) ? 2 :	                6 )

#define TX_TIM(__PORT__)  		((__PORT__ == 0) ? TIM14 : 		TIM15 )
#define TX_TIMCH(__PORT__) 	        ((__PORT__ == 0) ? TIM_CHANNEL_1 : 	TIM_CHANNEL_2 )
#define TX_TIM_GPIOPORT(__PORT__)       ( GPIOB )
#define TX_TIM_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_1 :		GPIO_PIN_15 )
#define TX_TIM_GPIOAF(__PORT__)         ((__PORT__ == 0) ? GPIO_AF0_TIM14 :	GPIO_AF1_TIM15 )
  
#define RX_TIM(__PORT__)  		((__PORT__ == 0) ? TIM3 : 		TIM1 )
#define RX_TIMCH(__PORT__) 		( TIM_CHANNEL_1 )
#define RX_TIMCH_TIMIT(__PORT__)        ( TIM_IT_CC1 )
#define RX_TIM_DMA_CC(__PORT__)         ( TIM_DMA_CC1 )
#define RX_TIM_IRQN(__PORT__) 		((__PORT__ == 0) ? TIM3_IRQn :          TIM1_CC_IRQn )
#define RX_TIM_IRQ_PRIO(__PORT__) 	((__PORT__ == 0) ? (RX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO) : (RX_IRQ_PRIO + USBPD_LOW_IRQ_PRIO) )
#define RX_COMP(__PORT__)		((__PORT__ == 0) ? COMP1 :              COMP2 )
#define RX_COMPOUT(__PORT__)		((__PORT__ == 0) ? COMP_OUTPUT_TIM3IC1 : COMP_OUTPUT_TIM1IC1 )


#define RX_CC1_GPIOPORT(__PORT__)	( GPIOA )
#define RX_CC1_PIN(__PORT__)		((__PORT__ == 0) ? GPIO_PIN_0 :		GPIO_PIN_2 )
#define RX_CC1_PIN_POSITION(__PORT__)	((__PORT__ == 0) ? 0 :		        2 )  
#define RX_CC1_ADCCH(__PORT__)		((__PORT__ == 0) ? ADC_CHANNEL_0 :	ADC_CHANNEL_4 )
#define RX_CC1_COMPCH(__PORT__)		( COMP_INVERTINGINPUT_IO1 )
#define RX_CC2_GPIOPORT(__PORT__)	( GPIOA )
#define RX_CC2_PIN(__PORT__)		((__PORT__ == 0) ? GPIO_PIN_5 :		GPIO_PIN_4 )
#define RX_CC2_PIN_POSITION(__PORT__)	((__PORT__ == 0) ? 5 :		        4 )  
#define RX_CC2_ADCCH(__PORT__)		((__PORT__ == 0) ? ADC_CHANNEL_2 :	ADC_CHANNEL_5 )
#define RX_CC2_COMPCH(__PORT__)		((__PORT__ == 0) ? COMP_INVERTINGINPUT_DAC2 : COMP_INVERTINGINPUT_DAC1 )
#define HRP_POSITION(__PORT__)          ((__PORT__ == 0) ? 12 :		        7 )      
    
#define RX_REF_GPIOPORT(__PORT__)	( GPIOA )
#define RX_REF_PIN(__PORT)		( GPIO_PIN_1 )
#define RX_REF_ADCCH(__PORT__)		( ADC_CHANNEL_1 )

 
  
/* MSP MACRO */

#define GET_PORT_FROM_I2C(hi2c) \
( (uint8_t)( hi2c->Instance == I2C2)? 0 : 1 )

#define I2C_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_I2C2_CLK_ENABLE(); \
                              /* else \
                                __HAL_RCC_I2C1_CLK_ENABLE(); */ \
                              } while(0)

#define I2C_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_I2C2_CLK_DISABLE(); \
                              /* else \
                                __HAL_RCC_I2C1_CLK_DISABLE(); */ \
                              } while(0)                                

#define GET_PORT_FROM_SPI(hspi) \
( (uint8_t)( hspi->Instance == TX_SPI(0) )? 0 : 1 )

#define SPI_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_SPI2_CLK_ENABLE(); \
                              /* else \
                                __HAL_RCC_SPI1_CLK_ENABLE(); */ \
                              } while(0)

#define SPI_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_SPI2_CLK_DISABLE(); \
                              /* else \
                                __HAL_RCC_SPI1_CLK_DISABLE(); */ \
                              } while(0)                                
                                
                                
#define DMA_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                  __HAL_RCC_DMA1_CLK_ENABLE(); \
                              /* else \
                                __HAL_RCC_DMA1_CLK_ENABLE(); */ \
                              } while(0)

                                
#define DMA_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                  __HAL_RCC_DMA1_CLK_DISABLE(); \
                              /* else \
                                __HAL_RCC_DMA1_CLK_DISABLE(); */ \
                              } while(0)
                                

                             

                                
#define IS_TX_TIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == TX_TIM(1)) ) )
#define IS_RX_TIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_TIM(0)) || (htim_base->Instance == RX_TIM(1)) ) )
#define IS_RX_COUNTTIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_COUNTTIM(0)) || (htim_base->Instance == RX_COUNTTIM(1)) ) )

#define GET_PORT_FROM_TIM(htim_base) \
( (uint8_t)( (htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == RX_TIM(0)) || (htim_base->Instance == RX_COUNTTIM(0)) )? 0 : 1 )




#define RX_COUNTTIM_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM16_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_ENABLE(); \
                              } while(0)

                                
#define RX_COUNTTIM_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM16_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM17_CLK_DISABLE(); \
                              } while(0)  
                                
                                
  /* Note (GN): To be reviewed */
                                

/* &251& */
#define TX_TIM_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM14_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM15_CLK_ENABLE(); \
                              } while(0)

                                
                                
/* &251& */
#define RX_TIM_CLK_ENABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM3_CLK_ENABLE(); \
                              else \
                                __HAL_RCC_TIM1_CLK_ENABLE(); \
                              } while(0)                                

                                 

/* &251& */
#define TX_TIM_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM14_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM15_CLK_DISABLE(); \
                              } while(0)

/* &251& */
#define RX_TIM_CLK_DISABLE(hport) do { \
                              if (hport==0) \
                                __HAL_RCC_TIM3_CLK_DISABLE(); \
                              else \
                                __HAL_RCC_TIM1_CLK_DISABLE(); \
                              } while(0)                                




#endif                                
                               
                                
  /**
  * @}
  */
  
  
#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PORTHANDLE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
