/**
  ******************************************************************************
  * @file    usbpd_cad.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains the headers of usbpd_cad.h for Cable Attach-Detach
  *          controls.
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
#ifndef __USBPD_CAD_H_
#define __USBPD_CAD_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_conf.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_CAD
  * @{
  */


/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_CAD_Exported_Types USBPD CORE CAD Exported Types
  * @{
  */
   
/** 
  * @brief funtion return value @ref USBPD_CORE_CAD
  * @{
  */
typedef enum
{
  USBPD_CAD_OK                  = 0x00,         /*!< USBPD CAD Status OK                */
  USBPD_CAD_INVALID_PORT        = 0x01,         /*!< USBPD CAD Status INVALID PORT      */
  USBPD_CAD_ERROR               = 0x02          /*!< USBPD CAD Status ERROR             */
}USBPD_CAD_StatusTypeDef;
/** 
  * @}
  */

/**
  * @brief activation value @ref USBPD_CORE_CAD
  * @{
  */
 typedef enum 
 {
   USBPD_CAD_DISABLE            = 0x00,         /*!< USBPD CAD activation status Disable   */
   USBPD_CAD_ENABLE             = 0x01          /*!< USBPD CAD activation status Enable   */
 } USBPD_CAD_activation;
/** 
  * @}
  */
 
 /**
   * @brief CAD State value @ref USBPD_CORE_CAD
   * @{
   */
 typedef enum 
 {
   USBPD_CAD_STATE_RESET        = 0x00,         /*!< USBPD CAD State Reset                              */
   USBPD_CAD_STATE_DETACHED     = 0x01,         /*!< USBPD CAD State No cable detected                  */ 
   USBPD_CAD_STATE_ATTACHED_WAIT= 0x02,         /*!< USBPD CAD State Port partner detected              */ 
   USBPD_CAD_STATE_ATTACHED     = 0x03,         /*!< USBPD CAD State Port partner attached              */ 
   USBPD_CAD_STATE_EMC          = 0x04,         /*!< USBPD CAD State Electronically Marked Cable detected   */ 
   USBPD_CAD_STATE_ATTEMC       = 0x05,         /*!< USBPD CAD State Port Partner detected throug EMC   */ 
   USBPD_CAD_STATE_ACCESSORY    = 0x06,         /*!< USBPD CAD State Accessory detected                 */ 
   USBPD_CAD_STATE_DEBUG        = 0x07,         /*!< USBPD CAD State Debug detected                     */ 
   USBPD_CAD_STATE_SWITCH_TO_SRC= 0x08,         /*!< USBPD CAD State switch to Source                   */ 
   USBPD_CAD_STATE_SWITCH_TO_SNK= 0x09,         /*!< USBPD CAD State switch to Sink                     */ 
   USPPD_CAD_STATE_UNKNOW       = 0x0A          /*!< USBPD CAD State unknow                             */
 } USBPD_CAD_STATE;
/** 
  * @}
  */ 

 /**
  * @brief CallBacks exposed by the @ref USBPD_CORE_CAD
  */
typedef struct
{
  /**
  * @brief  CallBack reporting events on a specified port.
  * @param  PortNum: The handle of the port
  * @param  State: CAD state
  * @param  Cc:	The Communication Channel for the USBPD communication
  * @retval None
  */
  void (*USBPD_CAD_CallbackEvent)(uint8_t PortNum, USBPD_CAD_STATE State, CCxPin_TypeDef Cc);
}USBPD_CAD_Callbacks;


/**
  * @brief  USBPD CAD handle Structure definition
  * @{
  */
typedef struct
{
  /* Template definition which is HW dependent */
  USBPD_CAD_activation        activation;              /*!< USBPD CAD activation status       */
  USBPD_CAD_STATE             state;                   /*!< USBPD CAD state                   */
  CCxPin_TypeDef              cc;                      /*!< USBPD CAD CCx                     */
  CAD_HW_Condition_TypeDef    OldHWCondtion;
  USBPD_CAD_Callbacks         callback;                /*!< USBPD CAD callback                */
}USBPD_CAD_HandleTypeDef;
/** 
  * @}
  */ 

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/** @defgroup USBPD_CORE_CAD_Exported_Defines USBPD CORE CAD Exported Defines
  * @{
  */
#define CAD_Task_delay                         2  /*!< USBPD CAD task delay   */    
/**
  * @}
  */
    
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_CORE_CAD_Exported_Functions
  * @{
  */
void                    USBPD_CAD_Process(void);
void                    USBPD_CAD_PortEnable(uint8_t PortNum, USBPD_CAD_activation act);
USBPD_CAD_StatusTypeDef USBPD_CAD_Init(uint8_t PortNum, USBPD_CAD_Callbacks CallbackFunctions);
void                    USBPD_CAD_AssertRd(uint8_t PortNum);
void                    USBPD_CAD_AssertRp(uint8_t PortNum);
void                    USBPD_CAD_TimerCounter(uint8_t PortNum);
void                    CAD_StateMachine(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum);

#ifdef USBPD_CLI
CCxPin_TypeDef USBPD_CAD_CLI_GetCCLine(uint8_t PortNum);
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

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_CAD_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

