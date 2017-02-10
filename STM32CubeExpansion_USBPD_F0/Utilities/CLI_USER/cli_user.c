/**
  ******************************************************************************
  * @file    cli_user.c
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   API for CLI.
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

#include "cli_api.h"

#ifdef USBPD_CLI
/* Includes ------------------------------------------------------------------*/
#include "cli_user.h"
#include "cli_commands.h"
#include "cli_console.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static xQueueHandle xQueueRx, xQueueTx; /** Queues to Rx/Tx from/to uart port */
xComPortHandle * pxPort = NULL; /** Handle of the uart port */
static portCHAR xRxBuffer[2]; /** Buffer to receive the chars */

/* Private function prototypes -----------------------------------------------*/
void prvSerialInit(xComPortHandle * pxPort, USART_TypeDef * usart, portLONG lBaudRate);
void prvSerialReceiveStart(void);

/* Exported variables --------------------------------------------------------*/
/* Const messages output for the command console. */
const char * const CLI_ConfigWelcomeMessage[CLI_WELCOME_MESSAGE_LEN] = { /** Welcome message string */
  "\r\nConsole v0.4 USBPD Application Demo\r\n",
  "Type help for a list of commands.\r\nSTMicroelectronics\r\n",
  "Copyright (c) 2017 - All Rights Reserved.\r\n"
};
const char * const CLI_ConfigEndOfOutputMessage = ">"; /** String after the message */
const char * const CLI_ConfigNewLine = "\r\n"; /** New line */

/**
 * @brief  Initialize the CLI
 * @usage  Create the Queue and storage the handle of the uart port 
 *         note: the peripheral have to be initialized externally
 *         before to call this function
 */ 
void CLI_Init(xComPortHandle * pxPortHandle)
{
  ( void )CLI_ConfigWelcomeMessage;
  ( void )CLI_ConfigEndOfOutputMessage;
  ( void )CLI_ConfigNewLine;
  
  /* Create the queues */
  xQueueTx = xQueueCreate( CLI_QUEUE_TX_LENGTH, ( CLI_OUTPUT_MAX_SIZE ) );
  configASSERT( xQueueTx );

  xQueueRx = xQueueCreate( CLI_QUEUE_RX_LENGTH, ( CLI_INPUT_MAX_SIZE ) );
  configASSERT( xQueueRx );

  pxPort = pxPortHandle;
  configASSERT( pxPort );
}
/**
 * @brief  Start the CLI tasks
 * @usage  Provide a start led management, console and command
 */ 
void CLI_Run()
{
  CLI_ConsoleStart(configMINIMAL_STACK_SIZE*2, osPriorityAboveNormal, xQueueTx, xQueueRx, pxPort);
  CLI_CommandStart(200, osPriorityNormal, xQueueRx, xQueueTx);

  /* start the uart receiver thread */
  prvSerialReceiveStart();
}

/**
 * @brief  Send to CLI an async message
 * @usage  
 */ 
void CLI_Async_Notify(char *string)
{
  CLI_ConsoleDirectTx(string, 0);
}

/**
 * @brief  enable the interrupt to receive chars
 */
__IO HAL_StatusTypeDef status;
void prvSerialReceiveStart(void)
{
  configASSERT(pxPort);
  status = HAL_UART_Receive_IT(pxPort, (uint8_t *)xRxBuffer, 1);
}

/**
* @brief  Get the Rx Queue
* @return the Queue handle to receive command from uart
*/ 
xQueueHandle CLI_GetRxQueue() 
{ 
  return xQueueRx; 
}
/**
 * @brief  Get the Tx Queue
 * @return the Queue handle to transmit string to uart
 */
xQueueHandle CLI_GetTxQueue() 
{ 
  return xQueueTx; 
}
/**
 * @brief  HAL Callback to notify a char from uart
 * @param  huart the uart handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  configASSERT(pxPort == huart);
  
  CLI_ConsoleRxCharFromISR(xRxBuffer[0]);
  prvSerialReceiveStart();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file.
   */
  huart->ErrorCode = 0;
  prvSerialReceiveStart();
}

#endif /* USBPD_CLI */
