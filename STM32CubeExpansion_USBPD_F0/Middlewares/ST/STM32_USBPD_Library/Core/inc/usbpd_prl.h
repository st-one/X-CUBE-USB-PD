/**
  ******************************************************************************
  * @file    usbpd_prl.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Header file for Protocol Layer module.
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

#ifndef __USBPD_PRL_H_
#define __USBPD_PRL_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
//#include "usbpd_timersserver.h"
#include "usbpd_def.h"
//#include "usbpd_conf.h"
#if USBPD_TCPM_MODULE_ENABLED
#include "usbpd_tcpm.h"
#else
#include "usbpd_phy.h"
#endif /* USBPD_TCPM_MODULE_ENABLED */

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_PRL
  * @{
  */

/* Exported defines --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_PRL_Exported_types USBPD CORE PRL Exported types 
  * @{
  */
/** @defgroup USBPD_MsgHeaderStructure_definition USB PD Message header Structure definition
  * @brief USB PD Message header Structure definition
  * @{
  */
typedef union
{
  uint16_t d16;
  struct
  {
    uint16_t MessageType :                  /*!< Message Header's message Type                      */
    4;
    uint16_t Reserved4 :                    /*!< Reserved                                           */
    1;
    uint16_t PortDataRole :                 /*!< Message Header's Port Data Role                    */
    1;                                    
    uint16_t SpecificationRevision :        /*!< Message Header's Spec Revision                     */
    2;       
    uint16_t PortPowerRole_CablePlug :      /*!< Message Header's Port Power Role/Cable Plug field  */
    1;       
    uint16_t MessageID :                    /*!< Message Header's message ID                        */
    3;    
    uint16_t NumberOfDataObjects :          /*!< Message Header's Number of data object             */
    3;                
    uint16_t Extended :                     /*!< Reserved                                           */
    1; 
  }
  b;
}USBPD_MsgHeader_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_ExtendedMsgHeadere_definition USB PD Message Extendedheader Structure definition
  * @brief USB PD Message Extendedheader Structure definition
  * @{
  */
typedef union
{
  uint16_t d16;
  struct
  {
    uint16_t DataSize:9;      /*!< Message ExtendedHeader's DataSize          */     
    uint16_t Reserved:1;      /*!< reserved                                   */
    uint16_t RequestChunk:1;  /*!< Message ExtendedHeader's request chunk     */
    uint16_t ChunkNumber:4;   /*!< Message ExtendedHeader's chunk number      */
    uint16_t Chunked:1;       /*!< Message ExtendedHeader's chunked           */
  }
  b;
}USBPD_ExtendedMsgHeader_TypeDef;
/**
  * @}
  */

/**
  * @brief CallBacks exposed by the @ref USBPD_CORE_PHY to the @ref USBPD_CORE_PRL
  * */
typedef struct
{
#if !defined(__PE_DUALPORT)
  /**
    * @brief  Reports that a message has been received on a specified port.
    * @param  PortNum       The handle of the port
    * @param  Type        The type of the message received
    * @param  pMsgHeader  Pointer on the message header 
    * @retval None
    * @note Received data are stored inside PortNum->pRxBuffPtr
    */
  void (*USBPD_PRL_MessageReceived)(uint8_t PortNum, USBPD_SOPType_TypeDef Type, USBPD_MsgHeader_TypeDef *pMsgHeader);
#else
  /**
    * @brief  Reports to the PRL that a Rx event (message received on a specified port)
    * @param  PortNum     The handle of the port
    * @param  SOPMsgType  The type of the message received
    * @param  pMsgHeader  Pointer on the RX MSG header
    * @retval None
    */
  void (*USBPD_PRL_PostReceiveEvent)(uint8_t PortNum, USBPD_SOPType_TypeDef SOPMsgType, USBPD_MsgHeader_TypeDef *pMsgHeader);

  /**
    * @brief  Reports to the PRL that a Rx event has been complete (goodcrc send complete).
    * @param  PortNum   The handle of the port
    * @retval None
    */
  void (*USBPD_PRL_PostReceiveEventComplete)(uint8_t PortNum);
#endif /*__PE_DUALPORT*/

  /**
    * @brief  Reports to the PRL that a Reset received from channel.
    * @param  PortNum    The handle of the port
    * @param  Type    The type of reset performed
    * @retval None
    */
  void (*USBPD_PRL_ResetIndication)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  Reports to the PE that a Bist operation has been completed.
    * @param  PortNum    The handle of the port
    * @param  bistmode BIST mode
    * @retval None
    */
  void (*USBPD_PRL_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);

  /**
    * @brief  Request the PE to send SoftReset.
    * @param  PortNum    The handle of the port
    * @retval None
    */
  void (*USBPD_PRL_RequestSoftReset)(uint8_t PortNum);
} USBPD_PRL_Callbacks;

/**
  * @}
  */

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_CORE_PRL_Exported_Functions
  * @{
  */

/** @addtogroup USBPD_CORE_PRL_Exported_Functions_Group1
  * @{
  */

/* Timer interface */
void USBPD_PRL_TimerCounter(uint8_t PortNum);

/**
  * @}
  */

/** @addtogroup USBPD_CORE_PRL_Exported_Functions_Group2
  * @{
  */

/* PE interface */
USBPD_StatusTypeDef USBPD_PRL_Init(uint8_t PortNum, USBPD_PRL_Callbacks cbs, uint8_t *pRxBuffer, USBPD_PortPowerRole_TypeDef role);
void                USBPD_PRL_SetHeader(uint8_t PortNum, USBPD_PortPowerRole_TypeDef PortPowerRole, USBPD_PortDataRole_TypeDef PortDataRole, uint16_t SpecificationRevision);
void                USBPD_PRL_SetHeaderPowerRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef PortPowerRole);
void                USBPD_PRL_SetHeaderDataRole(uint8_t PortNum, USBPD_PortDataRole_TypeDef PortDataRole);
void                USBPD_PRL_SetHeaderSpecification(uint8_t PortNum, uint16_t SpecificationRevision);
void                USBPD_PRL_DeInit(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_PRL_SendMessage(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint8_t MsgType, uint8_t *TxBuffer, uint16_t MsgSize);
USBPD_StatusTypeDef USBPD_PRL_ResetRequestProcess(uint8_t PortNum, USBPD_SOPType_TypeDef Type);
void                USBPD_PRL_ResetRequestReset(uint8_t PortNum);
void                USBPD_PRL_Reset(uint8_t PortNum);
USBPD_StatusTypeDef USBDPD_PRL_BistCarrierEyeMode(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);
#if defined(USBPD_TCPM_MODULE_ENABLED)
USBPD_StatusTypeDef USBDPD_PRL_BistCarrierEyeModeExit(uint8_t PortNum, USBPD_BISTMsg_TypeDef BistMode);
#endif /* USBPD_TCPM_MODULE_ENABLED */

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

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PRL_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

