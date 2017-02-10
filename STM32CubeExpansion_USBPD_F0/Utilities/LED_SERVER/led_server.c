/**
  ******************************************************************************
  * @file    led_server.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   LED server file provides services to control the LEDs in order to 
  *          show current status and error messages.
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

/* Includes ------------------------------------------------------------------*/
#include "led_server.h"
#include "cmsis_os.h"

#ifdef USBPD_LED_SERVER

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LED_THREAD_PRIORITY     osPriorityAboveNormal  /* Task priority */
#define LED_PRECISION           100                    /* LED precision (ms) */
#define LED_BLINK_MODE_PERIOD   20

/* LED status structure */
typedef struct {
  LED_BSP_TypeDef  Index;
  LED_Mode         Mode;
  LED_Mode         ModePrevious;
  uint16_t         Period;
  uint16_t         Count;
} LED_Status; 

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Status structure array */
static LED_Status Status[LED_INDEX_LEN]; 
/* LED task handle */
osThreadId xLedThreadId;
uint16_t GlobalPeriod = LED_BLINK_MODE_PERIOD; /* 2 sec */
uint16_t GlobalCount = 0x00;
uint32_t GlobalMap = 0x55;
/* Remapping order bits */
const uint8_t LedOrder[] = {0, 1, 2, 3, 4, 5}; 

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void prvLedThread(void const * argument);
static inline void prvLedStatusInit(LED_BSP_TypeDef Index);
inline void prvLedToogle(LED_BSP_TypeDef Index);
inline void prvLedWrite(LED_BSP_TypeDef Index, uint8_t Value);
inline void prvLedOn(LED_BSP_TypeDef Index);
inline void prvLedOff(LED_BSP_TypeDef Index);
inline void prvLed(LED_BSP_TypeDef Index, uint8_t Value);

/* Functions Definition ------------------------------------------------------*/

/**
  * @brief This function performs an init of the LED manager.
  * @retval None
  */
void Led_Init()
{
  uint8_t i = 0;
  for(i=0; i<LED_INDEX_LEN; i++)
  {
    prvLedStatusInit((LED_BSP_TypeDef)i);
  }
  GlobalCount = 0;
}

/**
 * @brief  Set the LED status.
 * @param  cIndex: Status Index
 * @param  cMode: New mode to be set
 * @param  sPeriod: Prdiod of time for toggling
 */
void Led_Set(LED_BSP_TypeDef Index, LED_Mode Mode, uint16_t Period)
{
  /* Check parameters */
  if (!LED_INDEX_IsValid(Index) || 
      !LED_MODE_IsValid(Mode) || 
      !LED_PERIOD_IsValid(Mode, Period) )
  {
    return;
  }

  LED_Status * pStatus = &Status[Index];
  pStatus->Mode = Mode;
  pStatus->Period = Period;
  pStatus->Count = 0;
  
  switch(Mode)
  {
  case LED_MODE_ON:
    prvLedOn(Index);
    break;
  case LED_MODE_OFF:
    prvLedOff(Index);
    break;
  case LED_MODE_BLINK:
    pStatus->Count = pStatus->Period;
  case LED_MODE_BLINK_MAP:
  case LED_MODE_BLINK_ROLE_DRP:
  case LED_MODE_BLINK_ROLE_SRC:
  case LED_MODE_BLINK_ROLE_SNK:
  case LED_MODE_BLINK_VBUS:
  case LED_MODE_BLINK_CC1:
  case LED_MODE_BLINK_CC2:
    if (xLedThreadId == NULL)
    {
      osThreadDef(LEDThread, prvLedThread, LED_THREAD_PRIORITY, 0, configMINIMAL_STACK_SIZE);
      xLedThreadId = osThreadCreate(osThread(LEDThread), NULL);
    }
    if (pStatus->ModePrevious != Mode)
    {
      prvLedOff(Index);
    }
    break;
  case LED_MODE_INVALID:
  default:
    break;
  }
  pStatus->ModePrevious = Mode;
}

/**
 * @brief  Set the LED status according to the map.
 * @param  cLedBitmap: the new map to be set
 */
void Led_OrderedSetBits(uint8_t LedBitmap)
{
  int i = 0;

  for(i = 0; i < LED_INDEX_LEN; i++)
  {
    Led_Set((LED_BSP_TypeDef)LedOrder[i], (((LedBitmap>>i) & 0x01) == 0x01 ? LED_MODE_ON : LED_MODE_OFF), 0);
  }
}

///**
// * @brief  Set the LED status according to the map.
// * @param  cLedBitmap: the new map to be set
// */
//static inline void Led_OrderedSetBits(uint8_t powerrole)
//{
//  LED_Mode mode = LED_MODE_BLINK_ROLE_SRC;
//  if (powerrole == LED_MODE_BLINK_ROLE_SNK) 
//  {
//    mode = 
//  }
//}


/**
 * @brief  Callback for the task to provide the blink.
 * @param  argument: Thread argument
 */
static void prvLedThread(void const * argument)
{
  LED_Status * pStatus = NULL;
  uint16_t half_period;
  uint8_t i=0;

  /* Infinite loop */
  for(;;)
  {
	/* Divide by 2 */
    half_period = GlobalPeriod>>1;
    for(i=0; i<LED_INDEX_LEN; i++)
    {
      pStatus = &Status[i];
      if (LED_MODE_IsBlinking(pStatus->Mode))
      {
        switch(pStatus->Mode)
        {
        case LED_MODE_BLINK:
          if ((pStatus->Count % (pStatus->Period>>1)) == 0)
          {
            prvLedToogle((LED_BSP_TypeDef)i);
          }
          pStatus->Count++;
          pStatus->Count%=pStatus->Period;
          break;
        case LED_MODE_BLINK_MAP:
          prvLed((LED_BSP_TypeDef)i, (GlobalMap >> GlobalCount) & 0x01);
          break;
        case LED_MODE_BLINK_ROLE_SRC:
          if (GlobalCount == 0) prvLedOn((LED_BSP_TypeDef)i);
          else if (GlobalCount == 2) 
		  {
			prvLedOff((LED_BSP_TypeDef)i);
		  }
          break;
        case LED_MODE_BLINK_ROLE_SNK:
          if (GlobalCount == 0 || GlobalCount == 3) 
		  {
			prvLedOn((LED_BSP_TypeDef)i);
		  }
          else if (GlobalCount == 1 || GlobalCount == 4) 
		  {
			prvLedOff((LED_BSP_TypeDef)i);
		  }
          break;
        case LED_MODE_BLINK_ROLE_DRP:
          if (GlobalCount == 0 || GlobalCount == 4 || GlobalCount == 8) 
		  {
			prvLedOn((LED_BSP_TypeDef)i);
		  }
          else if (GlobalCount == 2 || GlobalCount == 6 || GlobalCount == 10) 
		  {
			prvLedOff((LED_BSP_TypeDef)i);
		  }
          break;
        case LED_MODE_BLINK_VBUS:
          if (GlobalCount == 0) 
		  {
			prvLedOn((LED_BSP_TypeDef)i);
		  }
          else if (GlobalCount == 2) 
		  {
			prvLedOff((LED_BSP_TypeDef)i);          
		  }
          break;
        case LED_MODE_BLINK_CC1:
          if (GlobalCount == half_period) 
		  {
			prvLedOn((LED_BSP_TypeDef)i);
		  }
          else if (GlobalCount == (half_period+2)) 
		  {
			prvLedOff((LED_BSP_TypeDef)i);
		  }
          break;
        case LED_MODE_BLINK_CC2:
          if (GlobalCount == half_period || GlobalCount == (half_period+3)) 
		  {
			prvLedOn((LED_BSP_TypeDef)i);
		  }
          else if (GlobalCount == (half_period+1) || GlobalCount == (half_period+4))
		  {
			prvLedOff((LED_BSP_TypeDef)i);
		  }
          break;
        case LED_MODE_INVALID:
        case LED_MODE_OFF:
        case LED_MODE_ON:
        default:
          break;
        }
      }
    }
    GlobalCount++;
    GlobalCount %= GlobalPeriod;
    osDelay(LED_PRECISION);
  }
}

/**
 * @brief  Initialize the status structure of the LED.
 * @param  cIndex: State index
*/
static inline void prvLedStatusInit(LED_BSP_TypeDef Index)
{
  configASSERT(LED_INDEX_IsValid(Index));

  Status[Index].Index = Index;
  Status[Index].Mode = LED_MODE_OFF;
  Status[Index].Period = LED_PERIOD_DEFAULT;
  Status[Index].Count = 0;
}

/* Board Wrapping Features */
/**
 * @brief  Perform a LED toggle.
 * @param  cIndex: State index
 */
void prvLedToogle(LED_BSP_TypeDef Index)
{
  if (!LED_INDEX_IsValid(Index)) 
  {
	return;
  }
  USBPD_BSP_LED_Toggle(Index);
}

/**
 * @brief  Turn On/Off a LED.
 * @param  cIndex: State Index
 * @param  cValue: State new value
 */
void prvLedWrite(LED_BSP_TypeDef Index, uint8_t Value)
{
  if (!LED_INDEX_IsValid(Index)) 
  {
	return;
  }
  USBPD_BSP_LED_Set(Index, Value == LED_ON);
}

/**
 * @brief  Turn on a LED.
 * @param  cIndex: State Index
 */
void prvLedOn(LED_BSP_TypeDef Index)
{
  USBPD_BSP_LED_On(Index);
}

/**
 * @brief  Turn off a LED.
 * @param  cIndex: State Index
 */
void prvLedOff(LED_BSP_TypeDef Index)
{
  USBPD_BSP_LED_Off(Index);
}

/**
 * @brief  Turn on off a LED.
 * @param  cIndex: State Index
 */
void prvLed(LED_BSP_TypeDef Index, uint8_t Value)
{
  USBPD_BSP_LED_Set(Index, Value);
}

#endif /* USBPD_LED_SERVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
