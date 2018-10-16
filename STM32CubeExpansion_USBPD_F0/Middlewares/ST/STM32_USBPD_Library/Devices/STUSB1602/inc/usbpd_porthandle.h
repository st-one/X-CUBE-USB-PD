/**
  ******************************************************************************
  * @file    usbpd_porthandle.h
  * @author  MCD Application Team
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
#include "usbpd_core.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_HW_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Types USBPD DEVICE PORTHANDLE Exported Types
  * @{
  */

/**
  * @brief Default current limit of the power switches supplying VCONN on the CC pins
  */
typedef enum
{
  VConn_Ilim_350mA = 0x00,  /*!< VCONN current limit set to 350 mA */
  VConn_Ilim_300mA = 0x01,  /*!< VCONN current limit set to 300 mA */
  VConn_Ilim_250mA = 0x02,  /*!< VCONN current limit set to 250 mA */
  VConn_Ilim_200mA = 0x03,  /*!< VCONN current limit set to 200 mA */
  VConn_Ilim_150mA = 0x04,  /*!< VCONN current limit set to 150 mA */
  VConn_Ilim_100mA = 0x05,  /*!< VCONN current limit set to 100 mA */
  VConn_Ilim_400mA = 0x06,  /*!< VCONN current limit set to 400 mA */
  VConn_Ilim_450mA = 0x07,  /*!< VCONN current limit set to 450 mA */
  VConn_Ilim_500mA = 0x08,  /*!< VCONN current limit set to 500 mA */
  VConn_Ilim_550mA = 0x09,  /*!< VCONN current limit set to 550 mA */
  VConn_Ilim_600mA = 0x0A,  /*!< VCONN current limit set to 600 mA */
} USBPD_VConnIlim_TypeDef;

/**
  * @brief SPI Mode for communication
  */
typedef enum
{
  STUSB16xx_SPI_Mode_TX = 0,  /*!< SPI used for transmission */
  STUSB16xx_SPI_Mode_RX = 1,  /*!< SPI used for reception    */
} STUSB1602_SPI_Mode_TypeDef;


/**
  * @brief Struct with parameter used in the decoding phase
  */
typedef struct
{
  uint8_t     exed_flag;              /*!< Exit cases flag              */
  uint8_t     preamble;               /*!< Flag set if preamble ended   */
  uint32_t    dataindex;              /*!< Index inside RxDataBuf       */
  uint32_t    dataoffset;             /*!< Offset inside RxDataBuf      */
  uint32_t    index;                  /*!< Word index inside RXBuf      */
  uint32_t    offset;                 /*!< Bit index inside RXBuf       */
} UnwrapData_TypeDef;

/**
  * @brief Status of decoding phase
  */
typedef enum
{
  USBPD_PHY_RX_STATUS_NONE,                   /*!< Unknown Status                   */
  USBPD_PHY_RX_STATUS_OK,                     /*!< Status OK                        */
  USBPD_PHY_RX_STATUS_SOP_DETECTING,          /*!< Detecting SOP                    */
  USBPD_PHY_RX_STATUS_DATA,                   /*!< Decoding Data                    */
  USBPD_PHY_RX_STATUS_MESSAGE_READY,          /*!< Message Ready                    */
  USBPD_PHY_RX_STATUS_ERROR,                  /*!< Generic error during decoding    */
  USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP,  /*!< Error; Unsupported SOP detected  */
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP,      /*!< Error: Invalid SOP detected      */
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL,   /*!< Error: Invalid symbol found      */
  USBPD_PHY_RX_STATUS_ERROR_EOP_NOT_FOUND,    /*!< Error: No EOP found              */
  USBPD_PHY_RX_STATUS_ERROR_CRC_FAILED,       /*!< Error: CRC failed                */
}
USBPD_PHY_RX_Status_TypeDef;

/**
  * @brief RX PHY Status
  */
typedef enum
{
  RX_OK,                    /*!< RX OK                        */
  RX_ERROR,                 /*!< RX ERROR                     */
  RX_ERROR_WRONG_ORDERSET,  /*!< RX ERROR: Wrong Ordered Set  */
  RX_ERROR_WRONG_SYMBOL,    /*!< RX ERROR: Wrong Symbol       */
  RX_ERROR_MISSING_EOP,     /*!< RX ERROR: Wrong EOP          */
  RX_ERROR_CRC_FAILED       /*!< RX ERROR: CRC Failed         */
} USBPD_PHY_RX_StatusTypeDef;

/**
  * @brief CallBacks exposed by the HW_IF to the PHY
  */
typedef struct
{
  /**
    * @brief  A message has been received on a specified port.
    * @param  PortNum   The current port number
    * @param  pPayload  pointer to the received data buffer in 5bit representation
    * @param  Bitsize   number of bit received
    * @retval RX PHY Status
    */
  USBPD_PHY_RX_StatusTypeDef (*USBPD_HW_IF_ReceiveMessage)(uint8_t PortNum, uint32_t *pPayload, uint16_t Bitsize);
  /**
    * @brief  The message transfer has been completed
    * @param  PortNum The current port number
    * @retval None
    */
  void (*USBPD_HW_IF_TxCompleted)(uint8_t PortNum);
  /**
    * @brief  Bist data sent callback from PHY_HW_IF
    * @param  PortNum   Index of current used port
    * @param  BistMode  Bist mode
    *         The supported mode are:
    *           @arg @ref USBPD_BIST_CARRIER_MODE0
    *           @arg @ref USBPD_BIST_CARRIER_MODE1
    *           @arg @ref USBPD_BIST_CARRIER_MODE2
    *           @arg @ref USBPD_BIST_CARRIER_MODE3
    *           @arg @ref USBPD_BIST_EYE_PATTERN
    * @retval None
    */
  void (*USBPD_HW_IF_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef BistMode);
  /**
    * @brief  A new message is incoming, need to reset the status.
    * @param  PortNum The current port number
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Reset)(uint8_t PortNum);
  /**
    * @brief  Some data are avaiable for current message, performe a decoding step.
    * @param  PortNum The current port number
    * @param  data    The last data received
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Accumulate)(uint8_t PortNum, uint32_t data);
  /**
    * @brief  The reception phase of the current message is completed, now to complete the decoding, check the message(CRC) and notify it.
    * @param  PortNum The current port number
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Completed)(uint8_t PortNum);
} USBPD_HW_IF_Callbacks;

/**
  * @brief USBPD Port State
  */
typedef enum
{
  HAL_USBPD_PORT_STATE_RESET             = 0x00,    /*!< Peripheral is not initialized                      */
  HAL_USBPD_PORT_STATE_READY             = 0x01,    /*!< Peripheral Initialized and ready for use           */
  HAL_USBPD_PORT_STATE_BUSY              = 0x02,    /*!< An internal process is ongoing                     */
  HAL_USBPD_PORT_STATE_BUSY_TX           = 0x03,    /*!< Data Transmission process is ongoing               */
  HAL_USBPD_PORT_STATE_BUSY_RX           = 0x04,    /*!< Data Reception process is ongoing                  */
  HAL_USBPD_PORT_STATE_WAITING           = 0x05,    /*!< Waiting for Data Reception process                 */
  HAL_USBPD_PORT_STATE_TIMEOUT           = 0x06,    /*!< Timeout state                                      */
  HAL_USBPD_PORT_STATE_ERROR             = 0x07,    /*!< Error                                              */
  HAL_USBPD_PORT_STATE_BIST              = 0x08     /*!< BIST Transmission process is ongoing               */
}HAL_USBPD_PORT_StateTypeDef;


/**
  * @brief  USBPD Port Handle
  */
typedef struct
{
  uint8_t                     Instance;        /*!< USBPD_PORT number                              */
  uint8_t                     *pTxBuffPtr;     /*!< Pointer to Tx Buffer                           */
  uint16_t                    TxXferSize;      /*!< Tx Transfer size                               */
  uint8_t                     *pRxBuffPtr;     /*!< Pointer to Raw Rx transfer Buffer              */
  uint32_t                    *pRxDataPtr;     /*!< Pointer to 5bdecoded data                      */
  uint16_t                    RxXferSize;      /*!< Rx Transfer size                               */
  CCxPin_TypeDef              CCx;             /*!< CC pin used for communication                  */
  FlagStatus                  CCxChange;       /*!< CC event change                                */
  HAL_LockTypeDef             Lock;            /*!< Locking object                                 */
  HAL_USBPD_PORT_StateTypeDef State;           /*!< Communication state                            */
  __IO uint32_t               ErrorCode;       /*!< Error code                                     */
  USBPD_PortPowerRole_TypeDef role;            /*!< The Role of the port Provider or Consumer      */
  uint32_t                    BIST_index;      /*!< Index for monitoring BIST Msg bits             */
  FunctionalState             VConn;           /*!< VConn status flag                              */
  USBPD_PortDataRole_TypeDef  DataRole;        /*!< Data role of the port                          */
  uint8_t                     TxSpareBits;     /*!< TxSpareBits                                    */
  UnwrapData_TypeDef          unwrapdata;      /*!< Fields used for decoding                       */
  SPI_HandleTypeDef           hspi;            /*!< SPI Handle parameters                          */
  DMA_HandleTypeDef           hdmatx;          /*!< Tx DMA Handle parameters                       */
  DMA_HandleTypeDef           hdmarx;          /*!< Rx DMA Handle parameters                       */
  I2C_HandleTypeDef           hi2c;            /*!< I2C Handle parameters                          */
  TIM_HandleTypeDef           htimcountrx;     /*!< Rx COUNTER TIM Handle parameters               */
  USBPD_HW_IF_Callbacks       cbs;             /*!< Port callbacks, see @ref USBPD_HW_IF_Callbacks */
  uint8_t                     AlertEventCount; /*!< Alert event counter                            */
  uint8_t                     CommLock;        /*!< CommLock                                       */
  uint8_t                     NbDetach;        /*!< Number of CC detach                            */
  uint8_t(*IsSwapOngoing)(uint8_t);            /*!< Function to check if a swap is ongoing used by @ref HW_IF_check_bus_idle */
  DEVICE_CUT_TypeDef          Device_cut;      /*!< Device Cut identifier                          */
  uint8_t                     ud_index_current[3];
  uint8_t                     modulo;
}STUSB16xx_PORT_HandleTypeDef;

/**
  * @brief  USBPD Initialization Structure
  */
typedef struct
{
  uint8_t                                     Instance;                 /*!< PORT number */
  USBPD_PortPowerRole_TypeDef                 RolePower;                /*!< The Port power role @ref USBPD_PortPowerRole_TypeDef */
  USBPD_PortDataRole_TypeDef                  RoleData;                 /*!< Port default data role @ref USBPD_PortDataRole_TypeDef */
  FunctionalState                             VendorMessages;           /*!< If enabled, It allows sending vendor messages */
  FunctionalState                             Ping;                     /*!< If enabled it allows sending Ping message when an Explicit Contract is established */
  FunctionalState                             ExtendedMessages;         /*!< If enabled it supports extended messages */
  uint16_t                                    PE_SCAP_HR;               /*!< Number of source capabilities requests before hard reset */
  Current_Capability_Advertised_TypeDef       CCCurrentAdvertised;      /*!< It advertises the current capability @ref Current_Capability_Advertised_TypeDef*/
  FunctionalState                             DataRoleSwap;             /*!< It enables or disables the data role swap capability */
  FunctionalState                             PowerRoleSwap;            /*!< It enables or disables the power role swap capability */
  FunctionalState                             VConnSwap;                /*!< It enables or disables the VCONN swap capability */
  FunctionalState                             VConnSupply;              /*!< It enables or disables the VCONN supply capability on CC pin */
  FunctionalState                             VConnDischarge;           /*!< It enables or disables the VCONN discharge on CC pin */
  FunctionalState                             VBusDischarge;            /*!< It enables or disables the Vbus discharge */
  USBPD_VConnIlim_TypeDef                     VConnIlim;                /*!< It allows changing the default current limit supplying VCONN on the CC pins */
  FunctionalState                             VConnMonitoring;          /*!< It enables or disables UVLO threshold detection on VCONN pin */
  VCONN_UVLO_Threshold_TypeDef                VConnThresholdUVLO;       /*!< High UVLO threshold of 4.65 V; 1b: Low UVLO threshold of 2.65 V (case of VCONN-powered accessories operating down to 2.7 V) @ref VCONN_UVLO_Threshold_TypeDef*/
  uint16_t                                    VBusSelect;               /*!< Value (mV) related to targeted VBUS voltage used for VBUS range monitoring by 100mV step */
  uint8_t                                     VbusVShiftHigh;           /*!< Shift coefficient used for computing the high threshold value (5% + VBUS_VSHIFT_HIGH) of the monitoring voltage range */
  uint8_t                                     VbusVShiftLow;            /*!< Shift coefficient used for computing the low threshold value (-5% - VBUS_VSHIFT_LOW) of the monitoring voltage range */
  FunctionalState                             VBusRange;                /*!< It enables or disables VBUS voltage range detection */
  VBUS_VSAFE0V_Threshold_TypeDef              VBusThresholdVSafe0V;     /*!< VBus vSafe0V threshold @ref VBUS_VSAFE0V_Threshold_TypeDef*/
  FunctionalState                             VddOVLO;                  /*!< It enables or disables OVLO threshold detection on Vdd voltage */
  FunctionalState                             VddUVLO;                  /*!< It enables or disables UVLO threshold detection on Vdd voltage */
  uint8_t                                     VBusDischargeTimeTo0V;    /*!< Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to 0 V: 84 ms*TDISPARAM (840ms is default discharge time) */
  uint8_t                                     VBusDischargeTimeToPDO;   /*!< Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to PDO: 20 ms*TDISPARAM (200 ms is default discharge time) */
  FunctionalState                             PowerAccessoryDetection;  /*!< It enables or disables powered accessory detection */
  FunctionalState                             PowerAccessoryTransition; /*!< It enables or disables powered accessory transition from Powered.Accessory state to Try.SNK */
}USBPD_Init_TypeDef;

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Defines USBPD DEVICE PORTHANDLE Exported Defines
  * @{
  */

#define USBPD_LOWEST_IRQ_PRIO   3   /*!< Lowest priority                    */
#define USBPD_LOW_IRQ_PRIO      1   /*!< High priority shift value          */
#define USBPD_HIGH_IRQ_PRIO     0   /*!< Low priority shift value           */
#define RX_IRQ_PRIO             0   /*!< Rx priority for first interrupt @note Communication is half duplex so @ref TX_IRQ_PRIO = @ref RX_IRQ_PRIO */
#define TX_IRQ_PRIO             0   /*!< Tx interrupt priority @note Communication is half duplex so @ref TX_IRQ_PRIO = @ref RX_IRQ_PRIO */
#define DMA_TIME_ELAPSED        60   /*!< DMA Time Elapsed value [us]       */
#define DMA_TIME_COUNT_COMPARE  10   /*!< DMA Time Count Compare value [us] */
#define DMA_TIME_TASK           20   /*!< DMA Time Task [us]                */

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Macros USBPD DEVICE PORTHANDLE Exported Macros
  *   @note @param __PORT__ identifies the Port number
  * @{
  */

#ifdef MB1303
/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_IOs STUSB1602 IOs
  *   @brief All those macro are used to define the HW setup on MCU side
  * @{
  */

/* GPIOs */
#define TX_EN_GPIO_PORT(__PORT__)           ( GPIOC )                                       /*!< TX_EN GPIO Port           */
#define TX_EN_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_2 : GPIO_PIN_3 )    /*!< TX_EN GPIO Pin            */
#define RESET_GPIO_PORT(__PORT__)           ( GPIOC )                                       /*!< RESET GPIO Port           */
#define RESET_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_6 : GPIO_PIN_7 )    /*!< RESET GPIO PIN            */
#define A_B_Side_GPIO_PORT(__PORT__)        ( GPIOA )                                       /*!< A_B Side GPIO Port        */
#define A_B_Side_GPIO_PIN(__PORT__)         ((__PORT__ == 0) ? GPIO_PIN_0 : GPIO_PIN_3 )    /*!< A_B Side GPIO Pin         */
#define ALERT_PORT_INDEX(__PORT__)          ((__PORT__ == 0) ? 0 : 1 )                      /*!< ALERT Pin index           */
#define ALERT_GPIO_PORT(__PORT__)           ( GPIOA )                                       /*!< ALERT GPIO Port           */
#define ALERT_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_1 : GPIO_PIN_2 )    /*!< ALERT GPIO Pin            */
#define ALERT_GPIO_IRQHANDLER(__PORT__)     ((__PORT__ == 0) ? EXTI0_1_IRQn : EXTI2_3_IRQn )/*!< ALERT GPIO IRQn           */
#define ALERT_GPIO_IRQPRIORITY(__PORT__)    ( USBPD_LOW_IRQ_PRIO )                          /*!< ALERT IRQ priority        */

/* I2C */
#ifndef __VVAR
#define I2C_INSTANCE(__PORT__)              ( I2C2 )                    /*!< I2C: Device           */
#define I2C_TIMING(__PORT__)                ( 0x20100917 )              /*!< I2C: Timing value     */
#define I2C_PORT(__PORT__)                  ( GPIOB )                   /*!< I2C: GPIO Port        */
#define I2C_SCL_PIN(__PORT__)               ( GPIO_PIN_10 )             /*!< I2C: SCL Pin          */
#define I2C_SCL_PORT(__PORT__)              ( GPIOB )                   /*!< I2C: SCL Port         */
#define I2C_SDA_PIN(__PORT__)               ( GPIO_PIN_11 )             /*!< I2C: SDA Pin          */
#define I2C_SDA_PORT(__PORT__)              ( GPIOB )                   /*!< I2C: SDA Port         */
#define I2C_MODE(__PORT__)                  ( GPIO_MODE_AF_OD )         /*!< I2C: GPIO Mode        */
#define I2C_PULL(__PORT__)                  ( GPIO_NOPULL )             /*!< I2C: Pull setup       */
#define I2C_SPEED(__PORT__)                 ( GPIO_SPEED_FREQ_HIGH )    /*!< I2C: GPIO Speed       */
#define I2C_ALTERNATE(__PORT__)             ( GPIO_AF1_I2C2 )           /*!< I2C: GPIO AF          */

#else
#define I2C_INSTANCE(__PORT__)                  ((__PORT__ == 0) ? I2C2  : I2C1)                   /*!< I2C: Device           */
#define I2C_TIMING(__PORT__)                    ((__PORT__ == 0) ? 0x20100917 : 0x20100917 )       /*!< I2C: Timing value     */
#define I2C_PORT(__PORT__)                      ( GPIOB )                                          /*!< I2C: GPIO Port        */
#define I2C_SCL_PIN(__PORT__)                   ( (__PORT__ == 0) ? GPIO_PIN_10 : GPIO_PIN_8 )     /*!< I2C: SCL Pin          */
#define I2C_SCL_PORT(__PORT__)                  ( GPIOB )                                          /*!< I2C: SCL Port         */
#define I2C_SDA_PIN(__PORT__)                   ( (__PORT__ == 0) ? GPIO_PIN_11 : GPIO_PIN_9 )     /*!< I2C: SDA Pin          */
#define I2C_SDA_PORT(__PORT__)                  ( GPIOB)                                           /*!< I2C: SDA Port         */
#define I2C_MODE(__PORT__)                      ( GPIO_MODE_AF_OD )                                /*!< I2C: GPIO Mode        */
#define I2C_PULL(__PORT__)                      ( GPIO_NOPULL )                                    /*!< I2C: Pull Setup       */
#define I2C_SPEED(__PORT__)                     ( GPIO_SPEED_FREQ_HIGH )                           /*!< I2C: GPIO Speed       */
#define I2C_ALTERNATE(__PORT__)                 ( (__PORT__ == 0) ? GPIO_AF1_I2C2 : GPIO_AF1_I2C1) /*!< I2C: GPIO Alternate Function */
#endif

/* SPI */
#define SPI_Instance(__PORT__)                  ((__PORT__ == 0) ? SPI2                  : SPI1 )                   /*!< SPI: Device           */
#define SPI_NSS_PORT(__PORT__)                  ((__PORT__ == 0) ? GPIOB                 : GPIOA )                  /*!< SPI: NSS Port         */
#define SPI_NSS_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_12           : GPIO_PIN_15 )            /*!< SPI: NSS Pin          */
#define SPI_NSS_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF0_SPI2         : GPIO_AF0_SPI1 )          /*!< SPI: NSS Alternate Function */
#define SPI_NSS_LL_APB(__PORT__)                ( LL_APB1_GRP2_PERIPH_SYSCFG )                                      /*!< SPI: NSS LL Group     */
#define SPI_NSS_LL_PORT(__PORT__)               ((__PORT__ == 0) ? LL_SYSCFG_EXTI_PORTB  : LL_SYSCFG_EXTI_PORTA )   /*!< SPI: NSS LL Port      */
#define SPI_NSS_LL_SYS_EXTI(__PORT__)           ((__PORT__ == 0) ? LL_SYSCFG_EXTI_LINE12 : LL_SYSCFG_EXTI_LINE15 )  /*!< SPI: NSS LL CFG EXTI Line */
#define SPI_NSS_LL_EXTI(__PORT__)               ((__PORT__ == 0) ? LL_EXTI_LINE_12 : LL_EXTI_LINE_15 )              /*!< SPI: NSS LL EXTI Line */
#define SPI_NSS_LL_IRQHANDLER(__PORT__)         ( EXTI4_15_IRQn )                                                   /*!< SPI: NSS LL IRQn      */
#define SPI_NSS_LL_IRQPRIORITY(__PORT__)        ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO )                            /*!< SPI: NSS LL IRQ priority */
#define SPI_CLK_PORT(__PORT__)                  ( GPIOB )                                                           /*!< SPI: CLK Port         */
#define SPI_CLK_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_13           : GPIO_PIN_3 )             /*!< SPI: CLK Pin          */
#define SPI_CLK_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF0_SPI2         : GPIO_AF0_SPI1 )          /*!< SPI: CLK Alternate Function */
#define SPI_MISO_PORT(__PORT__)                 ( GPIOB )                                                           /*!< SPI: MISO Port        */
#define SPI_MISO_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_14           : GPIO_PIN_4 )             /*!< SPI: MISO Pin         */
#define SPI_MISO_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF0_SPI2         : GPIO_AF0_SPI1 )          /*!< SPI: MISO Alternate Function */
#define SPI_MOSI_PORT(__PORT__)                 ( GPIOB )                                                           /*!< SPI: MOSI Port        */
#define SPI_MOSI_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_15           : GPIO_PIN_5 )             /*!< SPI: MOSI Pin         */
#define SPI_MOSI_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF0_SPI2         : GPIO_AF0_SPI1 )          /*!< SPI: MOSI Alternate Function */

/* DMA */
#define TX_DMACH(__PORT__)                      ((__PORT__ == 0) ? DMA1_Channel5            : DMA1_Channel3 )       /*!< DMA: Tx Channel       */
#define RX_DMACH(__PORT__)                      ((__PORT__ == 0) ? DMA1_Channel4            : DMA1_Channel2 )       /*!< DMA: Rx Channel       */
#define DMACHIRQ(__PORT__)                      ((__PORT__ == 0) ? DMA1_Channel4_5_6_7_IRQn : DMA1_Channel2_3_IRQn) /*!< DMA: IRQn             */
#define DMACHIRQ_PRIO(__PORT__)                 ( (TX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO))                              /*!< DMA: IRQ priority     */

/* Timers used during decoding phase */
#define RX_COUNTTIM(__PORT__)                   ((__PORT__ == 0) ? TIM16 : TIM17 )                                  /*!< COUNTTIM: Timer used during decoding phase */
#define RX_COUNTTIM_IRQN(__PORT__)              ((__PORT__ == 0) ? TIM16_IRQn : TIM17_IRQn )                        /*!< COUNTTIM: IRQn                 */
#define RX_COUNTTIMIRQ_PRIO(__PORT__)           ((__PORT__ == 0) ? (RX_IRQ_PRIO + USBPD_LOW_IRQ_PRIO) : (RX_IRQ_PRIO + USBPD_LOW_IRQ_PRIO) )  /*!< COUNTTIM: IRQ priority */
#define RX_COUNTTIMCH(__PORT__)                 ( TIM_CHANNEL_1 )                                                   /*!< COUNTTIM: Timer channel clocking decoding  */
#define RX_COUNTTIMCH_TIMIT(__PORT__)           ( TIM_IT_CC1 )                                                      /*!< COUNTTIM: Timer interrupt                  */
#define RX_COUNTTIMCH_ITFLAG(__PORT__)          ( TIM_FLAG_CC1 )                                                    /*!< COUNTTIM: Timer interrupt flag             */
/**
  * @}
  */

/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_MSP_MACRO MSP MACRO
  * @brief MACRO used by HAL MSP function to identify peripherals
 * @{
 */
#define GET_PORT_FROM_I2C(hi2c) \
( (uint8_t)( hi2c->Instance == I2C2)? 0 : 1 )   /*!< GET Port Number from I2C peripheral   */

#define I2C_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_I2C2_CLK_ENABLE(); \
else \
  __HAL_RCC_I2C1_CLK_ENABLE();  \
} while(0)                                      /*!< Enables Clock of I2C peripheral for the specified port  */

#define I2C_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_I2C2_CLK_DISABLE(); \
else \
  __HAL_RCC_I2C1_CLK_DISABLE();  \
} while(0)                                      /*!< Disables Clock of I2C peripheral for the specified port */

#define GET_PORT_FROM_SPI(hspi) \
( (uint8_t)( hspi->Instance == SPI_Instance(0) )? 0 : 1 )  /*!< GET Port Number from SPI peripheral   */

#define SPI_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_CLK_ENABLE(); \
else \
  __HAL_RCC_SPI1_CLK_ENABLE();  \
} while(0)                                      /*!< Enables Clock of SPI peripheral for the specified port  */

#define SPI_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_CLK_DISABLE(); \
else \
  __HAL_RCC_SPI1_CLK_DISABLE();  \
} while(0)                                      /*!< Disables Clock of SPI peripheral for the specified port */

#define DMA_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_DMA1_CLK_ENABLE(); \
else \
  __HAL_RCC_DMA1_CLK_ENABLE();   \
} while(0)                                      /*!< Enables Clock of DMA peripheral for the specified port  */

#define DMA_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_DMA1_CLK_DISABLE(); \
else \
  __HAL_RCC_DMA1_CLK_DISABLE();   \
} while(0)                                      /*!< Disables Clock of DMA peripheral for the specified port */

#define IS_RX_COUNTTIM(htim_base) \
( (uint8_t) ( (htim_base->Instance == RX_COUNTTIM(0)) || (htim_base->Instance == RX_COUNTTIM(1)) ) ) /*!< Identifies COUNTTIM peripherals */

#define GET_PORT_FROM_TIM(htim_base) \
( (uint8_t)( (htim_base->Instance == RX_COUNTTIM(0)) )? 0 : 1 ) /*!< GET Port Number from TIM peripheral   */

#define RX_COUNTTIM_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_TIM16_CLK_ENABLE(); \
else \
  __HAL_RCC_TIM17_CLK_ENABLE(); \
} while(0)                                      /*!< Enables Clock of COUNTTIM peripheral for the specified port  */

#define RX_COUNTTIM_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_TIM16_CLK_DISABLE(); \
else \
  __HAL_RCC_TIM17_CLK_DISABLE(); \
} while(0)                                      /*!< Disables Clock of COUNTIM peripheral for the specified port */

/**
  * @}
  */

#else

/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_IOs STUSB1602 IOs
  * @brief All those macro are used to define the HW setup on MCU side
  * @{
  */

/* STUSB16xx EVAL IOs */
#define DISCH_PATH_SNK_PORT(__PORT__)       ( GPIOC )               /*!< Discharge Path SNK GPIO Port   */
#define DISCH_PATH_SNK_PIN(__PORT__)        ( GPIO_PIN_14 )         /*!< Discharge Path SNK GPIO Pin    */
#define DISCH_PATH_SRC_PORT(__PORT__)       ( GPIOC )               /*!< Discharge Path SRC GPIO Port   */
#define DISCH_PATH_SRC_PIN(__PORT__)        ( GPIO_PIN_8)           /*!< Discharge Path SRC GPIO Pin    */
#define VBUS_EN_SRC_STM32_PORT(__PORT__)    ( GPIOB)                /*!< Enable SRC GPIO Port           */
#define VBUS_EN_SRC_STM32_PIN(__PORT__)     ( GPIO_PIN_0 )          /*!< Enable SRC GPIO Pin            */

/* STUSB1602 IOs */
#define TX_EN_GPIO_PORT(__PORT__)           ( GPIOB )               /*!< TX_EN GPIO Port           */
#define TX_EN_GPIO_PIN(__PORT__)            ( GPIO_PIN_14 )         /*!< TX_EN GPIO Pin            */
#define RESET_GPIO_PORT(__PORT__)           ( GPIOC )               /*!< RESET GPIO Port           */
#define RESET_GPIO_PIN(__PORT__)            ( GPIO_PIN_6 )          /*!< RESET GPIO PIN            */
#define A_B_Side_GPIO_PORT(__PORT__)        ( GPIOA )               /*!< A_B Side GPIO Port        */
#define A_B_Side_GPIO_PIN(__PORT__)         ( GPIO_PIN_0 )          /*!< A_B Side GPIO Pin         */
#define ALERT_PORT_INDEX(__PORT__)          ( 0 )                   /*!< ALERT Pin index           */
#define ALERT_GPIO_PORT(__PORT__)           ( GPIOA )               /*!< ALERT GPIO Port           */
#define ALERT_GPIO_PIN(__PORT__)            ( GPIO_PIN_5 )          /*!< ALERT GPIO Pin            */
#define ALERT_GPIO_IRQHANDLER(__PORT__)     ( EXTI4_15_IRQn )       /*!< ALERT GPIO IRQn           */
#define ALERT_GPIO_IRQPRIORITY(__PORT__)    ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO )/*!< ALERT IRQ priority     */

/* I2C */
#define I2C_INSTANCE(__PORT__)              ( I2C2 )                /*!< I2C: Device                */
#define I2C_TIMING(__PORT__)                ( 0x20303E5D )          /*!< I2C: Timing value          */
#define I2C_PORT(__PORT__)                  ( GPIOB )               /*!< I2C: GPIO Port             */
#define I2C_SCL_PIN(__PORT__)               ( GPIO_PIN_10 )         /*!< I2C: SCL Pin               */
#define I2C_SCL_PORT(__PORT__)              ( GPIOB )               /*!< I2C: SCL Port              */
#define I2C_SDA_PIN(__PORT__)               ( GPIO_PIN_11 )         /*!< I2C: SDA Pin               */
#define I2C_SDA_PORT(__PORT__)              ( GPIOB )               /*!< I2C: SDA Port              */
#define I2C_MODE(__PORT__)                  ( GPIO_MODE_AF_OD )     /*!< I2C: GPIO Mode             */
#define I2C_PULL(__PORT__)                  ( GPIO_NOPULL )         /*!< I2C: Pull Setup            */
#define I2C_SPEED(__PORT__)                 ( GPIO_SPEED_FREQ_HIGH )/*!< I2C: GPIO Speed            */
#define I2C_ALTERNATE(__PORT__)             ( GPIO_AF1_I2C2 )       /*!< I2C: GPIO Alternate Function */

/* SPI */
#define SPI_Instance(__PORT__)              ((__PORT__ == 0) ? SPI2 : SPI1 )    /*!< SPI: Device    */
#define SPI_NSS_PORT(__PORT__)              ( GPIOB )               /*!< SPI: NSS Port              */
#define SPI_NSS_PIN(__PORT__)               ( GPIO_PIN_9 )          /*!< SPI: NSS Pin               */
#define SPI_NSS_ALTERNATE(__PORT__)         ( GPIO_AF5_SPI2 )       /*!< SPI: NSS Alternate Function*/
#define SPI_NSS_LL_APB(__PORT__)            ( LL_APB1_GRP2_PERIPH_SYSCFG )      /*!< SPI: NSS LL Group */
#define SPI_NSS_LL_PORT(__PORT__)           ( LL_SYSCFG_EXTI_PORTB )/*!< SPI: NSS LL Port           */
#define SPI_NSS_LL_SYS_EXTI(__PORT__)       ( LL_SYSCFG_EXTI_LINE9 )/*!< SPI: NSS LL CFG EXTI Line  */
#define SPI_NSS_LL_EXTI(__PORT__)           ( LL_EXTI_LINE_9 )      /*!< SPI: NSS LL EXTI Line      */
#define SPI_NSS_LL_IRQHANDLER(__PORT__)     ( EXTI4_15_IRQn )       /*!< SPI: NSS LL IRQn           */
#define SPI_NSS_LL_IRQPRIORITY(__PORT__)    ( RX_IRQ_PRIO + 1 + USBPD_LOW_IRQ_PRIO ) /*!< SPI: NSS LL IRQ priority */
#define SPI_CLK_PORT(__PORT__)              ( GPIOB )               /*!< SPI: CLK Port              */
#define SPI_CLK_PIN(__PORT__)               ( GPIO_PIN_13 )         /*!< SPI: CLK Pin               */
#define SPI_CLK_ALTERNATE(__PORT__)         ( GPIO_AF0_SPI2 )       /*!< SPI: CLK Alternate Function*/
#define SPI_MISO_PORT(__PORT__)             ( GPIOC )               /*!< SPI: MISO Port             */
#define SPI_MISO_PIN(__PORT__)              ( GPIO_PIN_2 )          /*!< SPI: MISO Pin              */
#define SPI_MISO_ALTERNATE(__PORT__)        ( GPIO_AF1_SPI2 )       /*!< SPI: MISO Alternate Function */
#define SPI_MOSI_PORT(__PORT__)             ( GPIOC )               /*!< SPI: MOSI Port             */
#define SPI_MOSI_PIN(__PORT__)              ( GPIO_PIN_3 )          /*!< SPI: MOSI Pin              */
#define SPI_MOSI_ALTERNATE(__PORT__)        ( GPIO_AF1_SPI2 )       /*!< SPI: MOSI Alternate Function */

/* DMA */
#define TX_DMACH(__PORT__)                  ( DMA1_Channel5 )       /*!< DMA: Tx Channel            */
#define RX_DMACH(__PORT__)                  ( DMA1_Channel4 )       /*!< DMA: Rx Channel            */
#define DMACHIRQ(__PORT__)                  ( DMA1_Channel4_5_6_7_IRQn)        /*!< DMA: IRQn       */
#define DMACHIRQ_PRIO(__PORT__)             ( (TX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO))  /*!< DMA: IRQ priority     */

/* Timers used during decoding phase */
#define RX_COUNTTIM(__PORT__)               ((__PORT__ == 0) ? TIM16 : TIM17 )              /*!< COUNTTIM: Timer used during decoding phase */
#define RX_COUNTTIM_IRQN(__PORT__)          ((__PORT__ == 0) ? TIM16_IRQn : TIM17_IRQn )    /*!< COUNTTIM: IRQn                             */
#define RX_COUNTTIMIRQ_PRIO(__PORT__)       ((__PORT__ == 0) ? (RX_IRQ_PRIO + USBPD_HIGH_IRQ_PRIO) : (RX_IRQ_PRIO + USBPD_LOW_IRQ_PRIO) )  /*!< COUNTTIM: IRQ priority            */
#define RX_COUNTTIMCH(__PORT__)             ( TIM_CHANNEL_1 )       /*!< COUNTTIM: Timer channel clocking decoding   */
#define RX_COUNTTIMCH_TIMIT(__PORT__)       ( TIM_IT_CC1 )          /*!< COUNTTIM: Timer interrupt                  */
#define RX_COUNTTIMCH_ITFLAG(__PORT__)      ( TIM_FLAG_CC1 )        /*!< COUNTTIM: Timer interrupt flag             */
/**
  * @}
  */

/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_MSP_MACRO MSP MACRO
  * @brief MACRO used by HAL MSP function to identify peripherals
 * @{
 */

#define GET_PORT_FROM_I2C(hi2c) \
    ( (uint8_t)( hi2c->Instance == I2C2)? 0 : 1 )   /*!< GET Port Number from I2C peripheral   */

#define I2C_CLK_ENABLE(PortNum) do { \
      __HAL_RCC_I2C2_CLK_ENABLE(); \
    } while(0)                                  /*!< Enables Clock of I2C peripheral for the specified port  */
      
#define I2C_CLK_DISABLE(PortNum) do { \
      __HAL_RCC_I2C2_CLK_DISABLE(); \
    } while(0)                                  /*!< Disables Clock of I2C peripheral for the specified port */

#define GET_PORT_FROM_SPI(hspi) \
      ( (uint8_t)( hspi->Instance == TX_SPI(0) )? 0 : 1 )   /*!< GET Port Number from SPI peripheral   */

#define SPI_CLK_ENABLE(PortNum) do { \
      __HAL_RCC_SPI2_CLK_ENABLE(); \
    } while(0)                                  /*!< Enables Clock of SPI peripheral for the specified port  */

#define SPI_CLK_DISABLE(PortNum) do { \
      __HAL_RCC_SPI2_CLK_DISABLE(); \
    } while(0)                                  /*!< Disables Clock of SPI peripheral for the specified port */

#define DMA_CLK_ENABLE(PortNum) do { \
      __HAL_RCC_DMA1_CLK_ENABLE(); \
    } while(0)                                  /*!< Enables Clock of DMA peripheral for the specified port  */

#define DMA_CLK_DISABLE(PortNum) do { \
      __HAL_RCC_DMA1_CLK_DISABLE(); \
    } while(0)                                  /*!< Disables Clock of DMA peripheral for the specified port */

#define IS_RX_COUNTTIM(htim_base) \
    ( (uint8_t) ( (htim_base->Instance == RX_COUNTTIM(0)) || (htim_base->Instance == RX_COUNTTIM(1)) ) ) /*!< Identifies COUNTTIM peripherals */

#define GET_PORT_FROM_TIM(htim_base) \
    ( (uint8_t)( (htim_base->Instance == TX_TIM(0)) || (htim_base->Instance == RX_TIM(0)) || (htim_base->Instance == RX_COUNTTIM(0)) )? 0 : 1 ) /*!< GET Port Number from TIM peripheral   */

#define RX_COUNTTIM_CLK_ENABLE(PortNum) do { \
    if (PortNum==0) \
      __HAL_RCC_TIM16_CLK_ENABLE(); \
    else \
      __HAL_RCC_TIM17_CLK_ENABLE(); \
} while(0)                                      /*!< Enables Clock of COUNTTIM peripheral for the specified port  */

#define RX_COUNTTIM_CLK_DISABLE(PortNum) do { \
    if (PortNum==0) \
      __HAL_RCC_TIM16_CLK_DISABLE(); \
    else \
      __HAL_RCC_TIM17_CLK_DISABLE(); \
} while(0)                                      /*!< Disables Clock of COUNTIM peripheral for the specified port */

#endif

/**
  * @}
  */

/**
  * @}
  */

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

