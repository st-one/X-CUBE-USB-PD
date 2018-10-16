/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.h
  * @author  MCD Application Team
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
#ifndef __USBPD_CAD_HW_IF_H_
#define __USBPD_CAD_HW_IF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_CAD_Exported_Types USBPD CORE CAD Exported Types
  * @{
  */
/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef enum
{
  USBPD_CAD_STATE_RESET        ,      /*!< USBPD CAD State Reset                                 */
  USBPD_CAD_STATE_DETACHED     ,      /*!< USBPD CAD State No cable detected                     */
  USBPD_CAD_STATE_ATTACHED_WAIT,      /*!< USBPD CAD State Port partner detected                 */
  USBPD_CAD_STATE_ATTACHED     ,      /*!< USBPD CAD State Port partner attached                 */
  USBPD_CAD_STATE_ATTACHED_LEGACY,    /*!< USBPD CAD State Port partner attached to legacy cable */
  USBPD_CAD_STATE_EMC          ,      /*!< USBPD CAD State Electronically Marked Cable detected  */
  USBPD_CAD_STATE_ATTEMC       ,      /*!< USBPD CAD State Port Partner detected throug EMC      */
  USBPD_CAD_STATE_ACCESSORY    ,      /*!< USBPD CAD State Accessory detected                    */
  USBPD_CAD_STATE_DEBUG        ,      /*!< USBPD CAD State Debug detected                        */
  USBPD_CAD_STATE_SWITCH_TO_SRC,      /*!< USBPD CAD State switch to Source                      */
  USBPD_CAD_STATE_SWITCH_TO_SNK,      /*!< USBPD CAD State switch to Sink                        */
  USBPD_CAD_STATE_SWITCH_TO_DETACHED, /*!< USBPD CAD State switch to Detached                    */
  USPPD_CAD_STATE_UNKNOW              /*!< USBPD CAD State unknow                                */
} USBPD_CAD_STATE;
/**
  * @}
  */

/**
  * @brief USB PD CC lines HW condition
  */
typedef enum
{
  HW_Detachment                         = 0x00,    /*!< Nothing attached                        */
  HW_Attachment                         = 0x01,    /*!< Sink attached                           */
  HW_PwrCable_NoSink_Attachment         = 0x02,    /*!< Powered cable without Sink attached     */
  HW_PwrCable_Sink_Attachment           = 0x03,    /*!< Powered cable with Sink or VCONN-powered Accessory attached   */
  HW_Debug_Attachment                   = 0x04,    /*!< Debug Accessory Mode attached           */
  HW_AudioAdapter_Attachment            = 0x05     /*!< Audio Adapter Accessory Mode attached   */
} CAD_HW_Condition_TypeDef;


/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef struct
{
  USBPD_SettingsTypeDef *settings;                                /*!< Settings for the system based                            */
  USBPD_ParamsTypeDef   *params;                                  /*!< Parameters definition                                    */
  void (*USBPD_CAD_WakeUp)(void);                                 /*!< Callback for the wakeup                                  */
  uint32_t tToogle_start;                                         /*!< Callback to check if there is a power role swap ongoing  */

  /* state bitmap */
  USBPD_CAD_STATE state: 4;                                       /*!< State of the CAD                                         */
  CCxPin_TypeDef  cc: 2;                                          /*!< CC line                                                  */
  CAD_HW_Condition_TypeDef CurrentHWcondition: 3;                 /*!< Current condition of the hardware                        */
  CAD_HW_Condition_TypeDef OldHWCondtion: 3;                      /*!< Previous condition of the hardware                       */
  CAD_SNK_Source_Current_Adv_Typedef SNK_Source_Current_Adv: 2;   /*!< Exposed resistance from source                           */
  uint32_t reserved: 18;                                          /*!< Reserved                                                 */
} CAD_HW_HandleTypeDef;
/** 
  * @}
  */ 

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions_Prototypes USBPD DEVICE CAD HW IF Exported Functions Prototypes
  * @{
  */
void      CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *Settings, USBPD_ParamsTypeDef *Params, void (*PtrWakeUp)(void));
uint32_t  CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX);
void CAD_Enter_ErrorRecovery(uint8_t PortNum);

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

#endif /* __USBPD_CAD_HW_IF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
