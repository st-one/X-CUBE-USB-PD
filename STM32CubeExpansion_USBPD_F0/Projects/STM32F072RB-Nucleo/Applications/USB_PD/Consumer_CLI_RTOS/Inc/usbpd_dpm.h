/**
  ******************************************************************************
  * @file    usbpd_dpm.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Header file for usbpd_dpm.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
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

#ifndef __USBPD_DPM_H_
#define __USBPD_DPM_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_conf.h"
#include "usbpd_def.h"
#include "usbpd_pe.h" 
#include "led_server.h"

/* Exported typedef ----------------------------------------------------------*/
   
/**
  * @brief  USBPD DPM handle Structure definition
  * @{
  */
typedef struct
{
  uint32_t                      DPM_ListOfRcvSRCPDO[USBPD_MAX_NB_PDO];   /*!< The list of received Source Power Data Objects from Port partner
                                                                              (when Port partner is a Source or a DRP port).                       */
  uint32_t                      DPM_NumberOfRcvSRCPDO;                   /*!< The number of received Source Power Data Objects from port Partner
                                                                              (when Port partner is a Source or a DRP port).
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
  uint32_t                      DPM_ListOfRcvSNKPDO[USBPD_MAX_NB_PDO];   /*!< The list of received Sink Power Data Objects from Port partner
                                                                              (when Port partner is a Sink or a DRP port).                         */
  uint32_t                      DPM_NumberOfRcvSNKPDO;                   /*!< The number of received Sink Power Data Objects from port Partner
                                                                              (when Port partner is a Sink or a DRP port).
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
  uint32_t                      DPM_RDOPosition;                         /*!< RDO Position of requested DO in Source list of capabilities          */
  uint32_t                      DPM_RequestedVoltage;                    /*!< Value of requested voltage                                           */
  uint32_t                      DPM_RequestDOMsg;                        /*!< Request Power Data Object message to be sent                         */
  uint32_t                      DPM_RcvRequestDOMsg;                     /*!< Received request Power Data Object message from the port Partner     */
  __IO uint8_t                  DPM_SNKRDOPosition;                      /*!< Position of the requested power data object in Sink port PDO list    */
  USBPD_SNKPowerRequest_TypeDef DPM_SNKRequestedPower;                   /*!< Requested Power by the sink board                                    */
  __IO uint32_t                 DPM_ErrorCode;                           /*!< USB PD Error code                                                    */
  __IO uint8_t                  DPM_IsConnected;                         /*!< USB PD connection state                                              */
  USBPD_PortPowerRole_TypeDef   DPM_PortPowerRole;                       /*!< USB PD current port power role, Sink or Source                       */
}USBPD_HandleTypeDef;

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
USBPD_StatusTypeDef USBPD_DPM_Init(void);
USBPD_StatusTypeDef USBPD_DPM_RequestNewPowerProfile(uint8_t PortNum, uint8_t PDOIndex);
void USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum);

#ifdef USBPD_CLI
USBPD_StatusTypeDef DPM_CLI_GetStatusInfo(uint8_t PortNum, uint8_t *pProfile, float *pVoltage, uint32_t *pCurrentPDO);
#endif /* USBPD_CLI */


#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DPM_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
