/**
  ******************************************************************************
  * @file    cli_console.h
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   Header file of CLI console 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to avoid recursive inclusion ---------------------------------------*/
#ifndef __CLI_CONSOLE_H
#define __CLI_CONSOLE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cli_api.h"
#include "cli_user.h"

/** @addtogroup CLI_Console_Main
  * @{
  */    
/* CLI Console interaction ****************************************************/
void CLI_ConsoleStart( uint16_t usStackSize, osPriority xPriority, xQueueHandle xQueueInParam, xQueueHandle xQueueOutParam, xComPortHandle * pxPortParam);
/**
  * @}
  */ 

/** @addtogroup CLI_Console_String
  * @{
  */  
/* CLI Console string manipulation ********************************************/
portBASE_TYPE CLI_ConsoleDirectRx(portCHAR * cText, portTickType xTicksToWait);
portBASE_TYPE CLI_ConsoleDirectTx(portCHAR * cText, portTickType xTicksToWait);
portBASE_TYPE CLI_ConsoleDirectTxFromISR(portCHAR * cText);
/**
  * @}
  */ 

/** @addtogroup CLI_Console_Char
  * @{
  */  
/* CLI Console char manipulation **********************************************/
portBASE_TYPE CLI_ConsoleRxCharFromISR(portCHAR cChar);
portBASE_TYPE CLI_ConsoleRxChar(portCHAR cChar, portTickType xTicksToWait);
portBASE_TYPE CLI_ConsoleRxChars(portCHAR * cText, portTickType xTicksToWait);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __CLI_CONSOLE_H */
