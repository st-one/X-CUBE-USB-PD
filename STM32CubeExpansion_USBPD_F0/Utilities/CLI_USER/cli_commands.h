/**
  ******************************************************************************
  * @file    cli_commands.h
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   Header file of CLI command 
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
#ifndef __CLI_COMMANDS_H
#define __CLI_COMMANDS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cli_user.h"
   
/** @addtogroup CLI_Command_Main
  * @{
  */
/* CLI Command interaction ****************************************************/
void CLI_CommandStart( uint16_t usStackSize, osPriority xPriority, xQueueHandle xQueueInParam, xQueueHandle xQueueOutParam );
void CLI_RegisterCommands( void );

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __CLI_COMMANDS_H */

