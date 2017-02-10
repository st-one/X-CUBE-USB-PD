/**
  ******************************************************************************
  * @file    cli_user.h
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   Header file of CLI API
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
#ifndef __CLI_USER_H
#define __CLI_USER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* FreeRTOS includes. */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../CLI_RTOS/FreeRTOS_CLI.h"

/* HAL */
#include "stm32f0xx_hal.h"

/* Macros --------------------------------------------------------------------*/
/**
 * @brief  Dimensions of the queues to be used for the communication 
 */
#define CLI_QUEUE_TX_LENGTH     5
#define CLI_QUEUE_RX_LENGTH	3

#define CLI_WELCOME_MESSAGE_LEN 3
   
#define DBG_MODULE  /* strip debug code at compile time */

/** @addtogroup CLI Debug Level
 * @{
 */  
/**
 * @brief  Debug level, the output message are displayed according with the 
 * debug level
 */
//#define __DBG_FORCE     -1
//#define __DBG_DISABLE   0 /* no message (output only response of the command) */
//#define __DBG_NORMAL    1 /* normal application message */
//#define __DBG_VERBOSE   2 /* testing information */
//#define __DBG_DEBUG     3 /* debug information */
//#define __DBG_IsValid(val) ( ((val) == __DBG_DISABLE) || ((val) == __DBG_NORMAL) || ((val) == __DBG_VERBOSE) || ((val) == __DBG_DEBUG) ) 
   
/* Exported variables --------------------------------------------------------*/
/**
 * @}
 */
/**
 * @brief  The text used as copyright
 */
//extern const char * const CLI_ConfigCopyright;
/**
 * @brief  The text used as welcome message
 */
extern const char * const CLI_ConfigWelcomeMessage[];
/**
 * @brief  The text after each command
 */
extern const char * const CLI_ConfigEndOfOutputMessage;
/**
 * @brief  The newline char
 */
extern const char * const CLI_ConfigNewLine;

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CLI API
 * @{
 */  

void prvSerialReceiveStart(void);

#ifdef DBG_MODULE
portCHAR CLI_GetDebugLevel( void );
void CLI_SetDebugLevel( portCHAR xDebugLevel );
#else
#define CLI_GetDebugLevel() 0
#define CLI_SetDebugLevel(xDebugLevel )
#endif

xQueueHandle CLI_GetRxQueue( void );
xQueueHandle CLI_GetTxQueue( void );
/**
 * @}
 */ 

#ifdef __cplusplus
}
#endif

#endif /* __CLI_USER_H */
