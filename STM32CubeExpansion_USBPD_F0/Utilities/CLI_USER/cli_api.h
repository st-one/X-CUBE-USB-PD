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
#ifndef __CLI_API_H
#define __CLI_API_H

/**
 * @brief  Include configuration from usbpd library
 */
#include "usbpd_conf.h"
/* USBPD_CLI have to be located between the usbpd_conf and the PE header file */
#ifdef USBPD_CLI 

/* Macros --------------------------------------------------------------------*/
/**
 * @brief  Dimensions the buffer into which input characters are placed.
 */
#define CLI_OUTPUT_MAX_SIZE configCOMMAND_INT_MAX_OUTPUT_SIZE 
#define CLI_INPUT_MAX_SIZE configCOMMAND_INT_MAX_OUTPUT_SIZE
//#define CLI_OUTPUT_MAX_SIZE 50
//#define CLI_INPUT_MAX_SIZE 50

/**
 * @brief  Definition of the handle type used for the communication
 */
#define xComPortHandle UART_HandleTypeDef

void CLI_Init( xComPortHandle * pxPortHandle);
void CLI_Run( void );
void CLI_Async_Notify(char *string);
#endif /* USBPD_CLI */

#endif /* __CLI_API_H */
