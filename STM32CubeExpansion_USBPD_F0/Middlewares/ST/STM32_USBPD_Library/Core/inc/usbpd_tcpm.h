/**
  ******************************************************************************
  * @file    usbpd_tcpm.h
  * @author  MCD Application Team
  * @brief   Header file containing functions prototypes of USBPD TCPM library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBPD_TCPM_H
#define __USBPD_TCPM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(USBPDCORE_TCPM_SUPPORT)

#include "string.h"
#include "usbpd_cad.h"
#include "usbpd_def.h"
#include "tcpc.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_TCPM
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USBPD_CORE_TCPM_Exported_Constants USBPD CORE TCPM Exported Types Constants
  * @{
  */
/**
  * @brief TCPM State value @ref USBPD_CORE_TCPM
  */
// typedef enum
// {
//   USBPD_TCPM_STATE_RESET,         /*!< USBPD TCPM State Reset                                */
//   USBPD_TCPM_STATE_DETACHED,      /*!< USBPD TCPM State No cable detected                    */
//   USBPD_TCPM_STATE_ATTACHED_WAIT, /*!< USBPD TCPM State Port partner detected                */
//   USBPD_TCPM_STATE_ATTACHED,      /*!< USBPD TCPM State Port partner attached                */
//   USBPD_TCPM_STATE_EMC,           /*!< USBPD TCPM State Electronically Marked Cable detected */
//   USBPD_TCPM_STATE_ATTEMC,        /*!< USBPD TCPM State Port Partner detected throug EMC     */
//   USBPD_TCPM_STATE_ACCESSORY,     /*!< USBPD TCPM State Accessory detected                   */
//   USBPD_TCPM_STATE_DEBUG,         /*!< USBPD TCPM State Debug detected                       */
//   USPPD_TCPM_STATE_UNKNOW         /*!< USBPD TCPM State Unknown                              */
// } USBPD_TCPM_STATE;

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_TCPM_Exported_TypesDefinitions USBPD CORE TCPM Exported Types Definitions
  * @{
  */
/**
  * @brief CallBacks exposed by the @ref USBPD_CORE_TCPM to the @ref USBPD_CORE_PRL
  */
typedef struct
{
  /**
    * @brief  Reports that a message has been received on a specified port.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of the message received
    * @retval None
    * @note Received data are stored inside hport->pRxBuffPtr
    */
  void (*USBPD_TCPM_MessageReceived)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PRL that a Reset received from channel.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of reset performed
    * @retval None
    */
  void (*USBPD_TCPM_ResetIndication)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PRL that a Reset operation has been completed.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of reset performed
    * @retval None
    */
  void (*USBPD_TCPM_ResetCompleted)(uint8_t hport, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports the TCPM layer discarded last message to sent because the bus is not idle.
    * @param  hport:    The handle of the port
    * @retval None
    * @note See Section 5.7 of USB Power Delivery specification Rev2, V1.1
    */
  void (*USBPD_TCPM_ChannelIdleAfterBusy)(uint8_t hport);

  /**
    * @brief  Reports to the PRL that a Bist operation has been completed.
    * @param  hport:    The handle of the port
    * @param  Type:    The type of Bist performed
    * @retval None
    */
  void (*USBPD_TCPM_BistCompleted)(uint8_t hport, USBPD_BISTMsg_TypeDef bistmode);

  /**
    * @brief  USB-PD message sent callback from TCPC
    * @param  PortNum port number value
    * @param  Status Status of the transmission
    * @retval None
    */
  void (*USBPD_TCPM_MessageReceivedTC)(uint8_t PortNum, uint32_t status);

} USBPD_PHY_Callbacks;

/**
  * @brief Initialization structure exposed by the @ref USBPD_CORE_TCPM to the @ref USBPD_CORE_PRL
  */
typedef struct
{
  uint8_t   *pRxBuffer;             /*!< Pointer to @ref USBPD_CORE_PRL RX Buffer for the current port */
  USBPD_PHY_Callbacks *pCallbacks;  /*!< TCPM Callbacks */
} USBPD_TCPM_HandleTypeDef;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_CORE_TCPM_Exported_Functions
  * @{
  */

/** @defgroup USBPD_CORE_TCPM_Exported_Functions_Grp2 USBPD CORE TCPM Exported Functions to PRL
  * @{
  */
/**
  * @brief  Initialize TCPC devices
  * @param  PortNum     Port number value
  * @param  pCallbacks  TCPM callbacks
  * @param  pRxBuffer   Pointer on the RX buffer
  * @param  PowerRole   Power role can be one of the following values:
  *         @arg @ref USBPD_PORTPOWERROLE_SNK
  *         @arg @ref USBPD_PORTPOWERROLE_SRC
  * @param  SupportedSOP  Supported SOP
  * @retval HAL status
  */
USBPD_StatusTypeDef  USBPD_PHY_Init(uint8_t PortNum, USBPD_PHY_Callbacks *pCallbacks, uint8_t *pRxBuffer, USBPD_PortPowerRole_TypeDef PowerRole, uint32_t SupportedSOP);

/**
  * @brief  Reset the PHY of a specified port.
  * @param  PortNum    Number of the port.
  * @retval None
  */
void                 USBPD_PHY_Reset(uint8_t PortNum);

/**
  * @brief  function to set the supported SOP
  * @param  PortNum       Number of the port.
  * @param  SOPSupported  List of the supported SOP
  * @retval None.
  */
void                 USBPD_PHY_SOPSupported(uint8_t PortNum,uint32_t SOPSupported);

/**
  * @brief  De-initialize TCPC devices
  * @param  PortNum Port number value
  * @retval None
  */
void                 USBPD_TCPM_DeInit(uint8_t Port);

/**
  * @brief  Get CC line for PD connection
  * @param  PortNum Port number value
  * @param  CC1_Level Pointer of status of the CC1 line
  * @param  CC2_Level Pointer of status of the CC2 line
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_get_cc(uint32_t Port, uint32_t *cc1, uint32_t *cc2);

/**
  * @brief  Set the polarity of the CC lines
  * @param  PortNum Port number value
  * @param  Polarity Polarity
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_polarity(uint32_t Port, uint32_t polarity);

/**
  * @brief  Set power and data role et PD message header
  * @param  PortNum      Port number value
  * @param  PowerRole Power role
  * @param  DataRole  Data role
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_msg_header(uint32_t Port, USBPD_PortPowerRole_TypeDef PowerRole, USBPD_PortDataRole_TypeDef DataRole);

/**
  * @brief  Enable or disable PD reception
  * @param  PortNum       Port number value
  * @param  State         Activation or deactivation of RX
  * @param  SupportedSOP  Supported SOP by PRL
  * @param  HardReset     Hard reset status based on @ref TCPC_hard_reset
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_set_rx_state(uint32_t PortNum, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State, uint32_t SupportedSOP, TCPC_hard_reset HardReset);

/**
  * @brief  Retrieve the PD message
  * @param  PortNum Port number value
  * @param  Payload Pointer on the payload
  * @param  Type Pointer on the message type
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_get_message(uint32_t Port, uint8_t *payload, uint8_t *Type);

/**
  * @brief  Transmit the PD message
  * @param  PortNum Port number value
  * @param  Type Message type
  * @param  pData Pointer on the data message
  * @param  RetryNumber Number of retry
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_transmit(uint32_t Port, USBPD_SOPType_TypeDef Type, const uint8_t *data, uint32_t RetryNumber);

/**
  * @brief  Send bist pattern.
  * @param  PortNum    Number of the port
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_PHY_Send_BIST_Pattern(uint32_t Port);

/**
  * @brief  Request a Reset on a specified port.
  * @param  PortNum   Number of the port
  * @param  Type      The type of reset (hard or cable reset).
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

/**
  * @brief  Request TCPC to enter a specific BIST test mode.
  * @param  PortNum  Port number value
  * @param  State    Enable BIST carrier mode 2
  * @retval USBPD status
  */
USBPD_StatusTypeDef  USBPD_TCPM_Send_BIST_Pattern(uint8_t Port, USBPD_FunctionalState State);

/**
 * @brief  function to set the SinkTxNg
 * @param  PortNum  Number of the port.
 * @retval none.
  */
void                 USBPD_PHY_SetResistor_SinkTxNG(uint8_t PortNum);

/**
 * @brief  function to set the SinkTxOK
 * @param  PortNum  Number of the port.
 * @retval none.
  */
void                 USBPD_PHY_SetResistor_SinkTxOK(uint8_t PortNum);

/**
 * @brief  function to check if SinkTxOK
 * @param  PortNum  Number of the port.
 * @retval USBPD status based on @ref USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef  USBPD_PHY_IsResistor_SinkTxOk(uint8_t PortNum);

/**
 * @brief  Trigger in Fast role swap signalling
 * @param  PortNum  Number of the port.
 * @retval None
  */
void                 USBPD_PHY_FastRoleSwapSignalling(uint8_t PortNum);

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

/**
  * @}
  */

#endif /* USBPDCORE_TCPM_SUPPORT */

#ifdef __cplusplus
}
#endif


#endif /* __USBPD_TCPM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
