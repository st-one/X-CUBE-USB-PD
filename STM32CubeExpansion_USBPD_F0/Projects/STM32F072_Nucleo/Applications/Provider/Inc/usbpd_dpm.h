/**
  ******************************************************************************
  * @file    usbpd_dpm.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    29-June-2016
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
  uint32_t                      DPM_ListOfPDO[USBPD_MAX_NB_PDO];         /*!< The list of supported Power Data Objects by the current port         */
  uint8_t                       DPM_NumberOfPDO;                         /*!< The number of supported Power Data Objects
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
  uint32_t                      DPM_ListOfRcvPDO[USBPD_MAX_NB_PDO];      /*!< The list of received Power Data Objects from the port Partner        */
  uint32_t                      DPM_NumberOfRcvPDO;                      /*!< The number of received Power Data Objects from the port Partner
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
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
void USBPD_DPM_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DPM_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
