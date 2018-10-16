/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains CAD interfaces functions.
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

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_trace.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_hw_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
extern  USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum);


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Handle for the ports inside @ref USBPD_DEVICE_HW_IF */
extern STUSB16xx_PORT_HandleTypeDef Ports[];

/**
  * @brief handle to manage the detection state machine 
  */
CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions USBPD DEVICE CAD HW IF Exported Functions
  * @{
  */


/**
  * @brief  This function performs an initialization of the CAD 
  * @param  PortNum        Number of the port
  * @param  Settings       Settings for the system based on @ref USBPD_SettingsTypeDef
  * @param  Params         Parameters definition based on @ref USBPD_ParamsTypeDef
  * @param  WakeUp         Callback for the wakeup
  * @param  IsSwapOngoing  Callback to check if there is a power role swap ongoing
  * @retval CC pin line based on @ref CCxPin_TypeDef
  */
void CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *Settings, USBPD_ParamsTypeDef *Params,  void (*WakeUp)(void))
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  
  /* store the settings and parameters */
  _handle->params = Params;
  _handle->settings = Settings;
  _handle->state = USBPD_CAD_STATE_RESET;
  _handle->cc = CCNONE;
  _handle->CurrentHWcondition = _handle->OldHWCondtion = HW_Detachment;
  _handle->SNK_Source_Current_Adv = vRd_Undefined;
  
  if(_handle->params->PE_PowerRole == USBPD_PORTPOWERROLE_SRC)
  {
    USBPDM1_DeAssertRd(PortNum);
    USBPDM1_AssertRp(PortNum);
  }
  else
  {
    USBPDM1_DeAssertRp(PortNum);
    USBPDM1_AssertRd(PortNum);
  }
}

/**
  * @brief  CAD enters in error recovery mode 
  * @param  PortNum  Number of the port
  * @retval None
  */ 
void CAD_Enter_ErrorRecovery(uint8_t PortNum)
{
  /* remove resistor terminaison 
     switch CAD_StateMachine to Error Recovery state
     wakeup CAD task */
  USBPD_HW_IF_ErrorRecovery(PortNum);
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

