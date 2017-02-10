/**
  ******************************************************************************
  * @file    usbpd_pe.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Header file of Policy Engine module.
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
#ifndef __USBPD_PE_H
#define __USBPD_PE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_prl.h"
#ifdef USBPD_VDM_ENABLED
#include "usbpd_vdm.h"
#endif
#include "string.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_PE
  * @{
  */

/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_CORE_PE_Exported_TypesDefinitions USBPD CORE Exported Types Definitions
  * @{
  */

/** @defgroup PE_CallBacks_structure_definition PE CallBacks structure definition
  * @brief  PE CallBacks exposed by the PE to the  DMP
  * @{
  */
typedef struct
{
  /**
    * @brief  Request the DPM to setup the new power level.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_PE_RequestSetupNewPower)(uint8_t PortNum);
  
  /**
    * @brief  Request the DPM to perform a HardReset.
    * @param  PortNum Port number
    * @retval None
  */
  uint32_t (*USBPD_PE_HardReset)(uint8_t PortNum);

  /**
    * @brief  Get evaluation of swap request from DPM.
    * @param  PortNum Port number
    * @retval USBPD_OK if PR_swap is possible, else USBPD_ERROR
  */
  USBPD_StatusTypeDef (*USBPD_PE_EvaluatPRSwap)(uint8_t PortNum);

  /**
    * @brief  Request the DPM to turn Off power supply.
    * @param  PortNum Port number
    * @param  Role Port role (Source or Sink)
    * @retval None
  */
  void (*USBPD_PE_TurnOffPower)(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);

  /**
    * @brief  Request the DPM to turn On power supply.
    * @param  PortNum Port number
    * @param  Role Port role (Source or Sink)
    * @retval None
  */
  void (*USBPD_PE_TurnOnPower)(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
  
  /**
    * @brief  Request the DPM to assert Rd.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_PE_AssertRd)(uint8_t PortNum);

  /**
    * @brief  Request the DPM to assert Rp.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_PE_AssertRp)(uint8_t PortNum);
  
  /**
    * @brief  Inform DPM that an Explicit contract is established.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_PE_ExplicitContractDone)(uint8_t PortNum);

  /**
    * @brief  Allow PE to retrieve information from DPM/PWR_IF.
    * @param  PortNum Port number
    * @param  DataId Type of data to be read from DPM
    *         This parameter can be one of the following values:
    *           @arg @ref USBPD_CORE_DATATYPE_SRC_PDO Source PDO reading requested
    *           @arg @ref USBPD_CORE_DATATYPE_SNK_PDO Sink PDO reading requested
    *           @arg @ref USBPD_CORE_DATATYPE_RCV_SRC_PDO Received Source PDO values reading requested
    *           @arg @ref USBPD_CORE_DATATYPE_RCV_SNK_PDO Received Sink PDO values reading requested
    *           @arg @ref USBPD_CORE_DATATYPE_REQ_VOLTAGE Requested voltage value reading requested
    *           @arg @ref USBPD_CORE_DATATYPE_REQUEST_DO  Request message DO reading requested (from Sink to Source)
    * @param  Ptr Pointer on address where DPM data should be written (u32 pointer)
    * @param  Size Pointer on nb of u32 written by DPM
    * @retval None
  */
  void (*USBPD_PE_GetDataInfo)(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t *Size);  

  /**
    * @brief  Allow PE to update information in DPM/PWR_IF.
    * @param  PortNum Port number
    * @param  DataId Type of data to be updated in DPM
    *         This parameter can be one of the following values:
    *           @arg @ref USBPD_CORE_DATATYPE_RDO_POSITION Storage of requested DO position in PDO list
    *           @arg @ref USBPD_CORE_DATATYPE_REQ_VOLTAGE Storage of requested voltage value
    *           @arg @ref USBPD_CORE_DATATYPE_RCV_SRC_PDO Storage of Received Source PDO values 
    *           @arg @ref USBPD_CORE_DATATYPE_RCV_SNK_PDO Storage of Received Sink PDO values
    *           @arg @ref USBPD_CORE_DATATYPE_RCV_REQ_PDO Storage of Received Sink Request PDO value
    *           @arg @ref USBPD_CORE_DATATYPE_REQUEST_DO  Storage of Request message DO (from Sink to Source)
    * @param  Ptr Pointer on address where DPM data to be updated could be read (u32 pointer)
    * @param  Size nb of u32 to be updated in DPM
    * @retval None
  */
  void (*USBPD_PE_SetDataInfo)(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t Size);

  /**
    * @brief  Callback to be used by PE to evaluate a Request from Sink
    * @param  PortNum Port number
    * @retval None
  */
  USBPD_StatusTypeDef (*USBPD_PE_EvaluateRequest)(uint8_t PortNum);

  /**
    * @brief  Callback to be used by PE to evaluate a Source Capabilities from Source
    * @param  PortNum Port number
    * @retval None
  */
  USBPD_StatusTypeDef (*USBPD_PE_EvaluateCapabilities)(uint8_t PortNum);

}USBPD_PE_Callbacks;

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_CORE_PE_Exported_Functions
  * @{
  */

/** @addtogroup USBPD_CORE_PE_Exported_Functions_Group1
  * @{
  */
USBPD_StatusTypeDef         USBPD_PE_Init(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role, USBPD_PE_Callbacks pecallbacks);
USBPD_StatusTypeDef         USBPD_PE_DeInit(uint8_t PortNum);
void                        USBPD_PE_Reset(uint8_t PortNum);
void                        USBPD_PE_SRCProcess(uint8_t PortNum);
void                        USBPD_PE_SNKProcess(uint8_t PortNum);
void                        USBPD_PE_DRPProcess(uint8_t PortNum);
USBPD_StatusTypeDef         USBPD_PE_IsCableConnected(uint8_t PortNum, uint8_t IsConnected);

/* PE Timer functions */
void                        USBPD_PE_TimerCounter(uint8_t PortNum);

/* Get current power role */
USBPD_PortPowerRole_TypeDef PE_GetPowerRole(uint8_t PortNum);
USBPD_PortPowerRole_TypeDef PE_GetCurrentPowerRole(uint8_t PortNum);
void                        PE_SetPowerRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role);
USBPD_PortDataRole_TypeDef  PE_GetDataRole(uint8_t PortNum);
USBPD_PortPowerRole_TypeDef PE_GetDefaultPortPowerRole(uint8_t PortNum);
USBPD_SpecRev_TypeDef       PE_GetSpecRevision(uint8_t PortNum);
uint8_t                     PE_GetSwapOngoing(uint8_t PortNum);
void                        PE_Reset(uint8_t PortNum);
void                        PE_ResetDuringSwap(uint8_t PortNum);

/*******************************************************************************
                              Power Negotiation
*******************************************************************************/

/* Request Message */
USBPD_StatusTypeDef         USBPD_PE_RequestNewPowerProfile(uint8_t PortNum, uint8_t PDOIndex);
USBPD_StatusTypeDef         USBPD_PE_RequestPowerRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef         USBPD_PE_GetReceivedPowerProfile(uint8_t PortNum, uint32_t *pPDO, uint32_t *pNbPDO);

#ifdef USBPD_VDM_ENABLED
USBPD_StatusTypeDef         USBPD_PE_SVDM_RequestIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef         USBPD_PE_SVDM_RequestModeExit(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef         USBPD_PE_SVDM_RequestAttention(uint8_t PortNum);
#endif

#ifdef USBPD_CLI
USBPD_StatusTypeDef         USBPD_PE_CLI_GetConnectionStatus(uint8_t PortNum, int8_t *pConnectionStatus);
USBPD_StatusTypeDef         USBPD_PE_CLI_GetCurrentRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef *pPortPowerRole, int8_t * pConnectionStatus);
USBPD_StatusTypeDef         USBPD_PE_CLI_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t *Size);
#endif
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

#endif /* __USBPD_PE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

