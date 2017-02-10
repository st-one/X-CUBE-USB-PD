/**
  ******************************************************************************
  * @file    cli_console.c
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   Console implementation, convert a flow char in a string newline
  *          terminated.
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
#include "cli_console.h"

/* Defines -------------------------------------------------------------------*/
#define MAX_MUTEX_WAIT	pdMS_TO_TICKS( 300 ) /** The maximum time to wait for 
                                                the mutex that guards the UART 
                                                to become available. */
#define ASCII_DEL	( 0x7F ) /** DEL acts as a backspace. */

/* Private variables ---------------------------------------------------------*/
//Handle definition
xQueueHandle xQueueIn;          /** Queue to receive the string (tx uart) */
xQueueHandle xQueueOut;         /** Queue to send the string (build internally) */
xQueueHandle xQueueRxChar;      /** Receiver chars to build the string */
osThreadId xRxThreadId;         /** Receiver task of the string */
osThreadId xTxThreadId;         /** Tnsmistter task of the string */

static SemaphoreHandle_t xTxMutex = NULL; /** used to sync several task on tx */
static xComPortHandle * pxPort; /** The handle to the com port */

/* Private function prototypes -----------------------------------------------*/
static void prvConsoleTxThread( void const * argument );
static void prvConsoleRxThread( void const * argument );
static void prvOutputLowChar(char);
static void prvOutputLowString( const char * const pcMessage );
static void prvSendSafeChar(char);
static void prvSendSafeString( const char * const pcMessage );

/*-----------------------------------------------------------*/


/**
 * @brief  API To perform a start of the Commands module.
 * @param  usStackSize       specify the stack size of the task
 * @param  uxPriority        specify the priority of the task
 * @param  xQueueInParam     input queue for strings to be tx through the uart
 * @param  xQueueOutParam    output queue for strings line terminated (i.e. output for CLI)
 * @param  pxPortParam       uart port already initialized
 */
void CLI_ConsoleStart( uint16_t usStackSize, 
                      osPriority xPriority,
                      xQueueHandle xQueueInParam, 
                      xQueueHandle xQueueOutParam, 
                      xComPortHandle * pxPortParam)
{
  	/* Initialise the UART. */
        pxPort = pxPortParam;
        configASSERT( pxPort );
        
	/* Create the semaphore used to access the UART Tx. */
	xTxMutex = xSemaphoreCreateMutex();
	configASSERT( xTxMutex );
        
        /* Create the queues */
        xQueueIn = xQueueInParam;
        xQueueOut = xQueueOutParam;
        
        xQueueRxChar = xQueueCreate( 30, 1 );
	configASSERT( xQueueRxChar );
        
        /* Create threads id for the console */
        osThreadDef(ConsoleTx, prvConsoleTxThread, xPriority, 0, usStackSize);
        xTxThreadId = osThreadCreate(osThread(ConsoleTx), NULL);
	configASSERT( xTxThreadId != NULL );
        
        osThreadDef(ConsoleRx, prvConsoleRxThread, xPriority, 0, usStackSize);
        xRxThreadId = osThreadCreate(osThread(ConsoleRx), NULL);
	configASSERT( xRxThreadId != NULL );
}

/**
 * @brief  Notify a char from an interrupt routine.
 * @param  cChar is the received char
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleRxCharFromISR(portCHAR cChar)
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  return xQueueSendToBackFromISR(
                          xQueueRxChar, 
                          &cChar, 
                          &xHigherPriorityTaskWoken);
}
/**
 * @brief  Notify a char. 
 * @param  cChar is the received char
 * @param  xTicksToWait The maximum amount of time the task should block waiting 
 *         for space to become available on the queue
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleRxChar(portCHAR cChar, portTickType xTicksToWait)
{
  return xQueueSendToBack(xQueueRxChar, &cChar, xTicksToWait);
}
/**
 * @brief  Notify an array of char.
 * @param  cText  is the array char (string)
 * @param  xTicksToWait The maximum amount of time the task should block waiting 
 *         for space to become available on the queue
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleRxChars(portCHAR * cText, portTickType xTicksToWait)
{
  int i = 0;
  portBASE_TYPE xResult = pdPASS;
  while(cText[i] != 0 && i < CLI_OUTPUT_MAX_SIZE && xResult == pdPASS)
  {
    xResult = CLI_ConsoleRxChar(cText[i], xTicksToWait);
    i++;
  }
  return xResult;
}

/**
 * @brief  Direct insert a string in the output queue
 * @param  cText  is the array char (string)
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleDirectRx(portCHAR * cText, portTickType xTicksToWait)
{
  portBASE_TYPE xReturn = pdFALSE;
  if (xQueueOut) 
  {
    xReturn = xQueueReceive( xQueueOut, cText, xTicksToWait);
  }
  return xReturn;
}
/**
 * @brief  Direct insert a string in the input queue
 * @param  cText  is the array char (string)
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleDirectTx(portCHAR * cText, portTickType xTicksToWait)
{
  portBASE_TYPE xReturn = pdFALSE;
  if (xQueueIn) 
  {
    xReturn = xQueueSendToBack( xQueueIn, cText, xTicksToWait);
  }
  return xReturn;
}
/**
 * @brief  Direct insert a string in the input queue
 * @param  cText  is the array char (string)
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 */
portBASE_TYPE CLI_ConsoleDirectTxFromISR(portCHAR * cText)
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  return xQueueSendToBackFromISR(
                          xQueueIn, 
                          cText, 
                          &xHigherPriorityTaskWoken);
}

/**
 * @brief  Callback for the Transmission
 * @param  pvParameters of the task 
 *         (see FreeRTOS documentation for more information)
 */
static void prvConsoleTxThread( void const * argument)
{
  //check err queue if item
  portBASE_TYPE xStatus;
  static char cText[ CLI_INPUT_MAX_SIZE ];
  for( ;; )
  {
    
    //retrieve the higher priority text to send through the serial port
    xStatus = xQueueReceive( xQueueIn, cText, portMAX_DELAY );
    if (xStatus == pdPASS)
    {
      prvSendSafeString(cText);
    }
  }
}
 
/**
 * @brief  Callback for the Reception
 * @param  pvParameters of the task 
 *         (see FreeRTOS documentation for more information)
 */
static void prvConsoleRxThread( void const * argument )
{
  signed char cRxedChar;
  portBASE_TYPE xStatus;
  unsigned char ucInputIndex = 0;
  static char cInputString[ CLI_INPUT_MAX_SIZE ], cLastInputString[ CLI_INPUT_MAX_SIZE ];

  for( ;; )
  {
    /* Wait for the next character.  The while loop is used in case
       INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
       be a genuine block time rather than an infinite block time. */
          
    xStatus = xQueueReceive( xQueueRxChar, &cRxedChar, portMAX_DELAY );
    if (xStatus == pdPASS)
    {
      /* Was it the end of the line? */
      if( cRxedChar == '\n' || cRxedChar == '\r' )
      {
        /* Just to space the output from the input. */
        prvSendSafeString(CLI_ConfigNewLine);

        /* See if the command is empty, nothing (output only the after command 
        message) */
        if( ucInputIndex == 0 )
        {
          cInputString[0] = '\0'; //empty string
        }

        /* Insert the received command in the output queue */
        xStatus = xQueueSendToBack( xQueueOut, cInputString, 0);
        if (xStatus != pdPASS)
        {
          /* a message error directly on output channel */
          xQueueSendToBack( xQueueIn, "Queue out full", 100);
        }

        /* Clear the input string ready to receive the next command.
        Remember the command that was just processed first in case it is
        to be processed again. */
        strcpy( cLastInputString, cInputString );
        ucInputIndex = 0;
        memset( cInputString, 0x00, CLI_INPUT_MAX_SIZE );
      }
      else
      {
        if( cRxedChar == '\r' )
        {
          /* Ignore the character. Nothing to do*/
        }
        else if( ( cRxedChar == '\b' ) || ( cRxedChar == ASCII_DEL ) )
        {
          /* Backspace was pressed.  Erase the last character in the string - 
          if any. */
          if( ucInputIndex > 0 )
          {
            /* Echo the character back. */
            prvSendSafeChar(cRxedChar);
            
            /*  remove the char */
            ucInputIndex--;
            cInputString[ ucInputIndex ] = '\0';
          }
        }
        else
        {
          /* A character was entered.  Add it to the string entered so
          far.  When a \n is entered the complete string will be
          passed to the command interpreter. */
          if( cRxedChar >= ' ' && cRxedChar <= '~' && ucInputIndex < CLI_INPUT_MAX_SIZE )
          {
            /* Echo the character back. */
            prvSendSafeChar(cRxedChar);

            /* Add the char to the buffer (lower case) */
            cInputString[ ucInputIndex ] = cRxedChar >= 'A'  && cRxedChar <= 'Z' ? cRxedChar | 0x20 : cRxedChar; //case insensitive
            ucInputIndex++;
          }
        }
      }

    } /* Queue receive */
    else 
    {
      osDelay(100);
    }
  } /* infinite loop - for */
}

/**
 * @brief  Raw send the char through the uart using the HAL.
 * @param  cChar char has to trasmit through the uart
 */
inline void prvOutputLowChar(char cChar)
{
  if (pxPort != NULL)
  {
    HAL_UART_Transmit(pxPort, (uint8_t *)&cChar, 1, 100);
  }
}
/**
 * @brief  Raw send the string through the uart using the HAL
 * @param  pcText string has to trasmit through the uart
 */
inline void prvOutputLowString(const char * const pcText)
{
  if (pxPort != NULL)
  {
    portCHAR len = (portCHAR)strlen(pcText);
    if (len > 0)
    {
      HAL_UART_Transmit(pxPort, (uint8_t *)pcText, len, 100);
    }
  }
}
/**
 * @brief  Send the char in safe manner, sync by a semaphore
 * @param  cChar char has to trasmit through the uart
 */
void prvSendSafeChar(char cChar)
{
  if( xSemaphoreTake(xTxMutex, MAX_MUTEX_WAIT ) == pdPASS)
  {
    prvOutputLowChar(cChar);
    xSemaphoreGive( xTxMutex );
  }
}
/**
 * @brief  Send the string in safe manner, sync by a semaphore
 * @param  pcText string has to trasmit through the uart
 */
void prvSendSafeString(const char * const pcText)
{
  if( xSemaphoreTake( xTxMutex, MAX_MUTEX_WAIT ) == pdPASS )
  {
    prvOutputLowString(pcText);
    xSemaphoreGive( xTxMutex );
  }
}

/*-----------------------------------------------------------*/
#endif /* USBPD_CLI */
