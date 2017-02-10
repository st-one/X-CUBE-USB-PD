/**
  ******************************************************************************
  * @file    led_server.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   Header file of LED server module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_SERVER_H
#define __LED_SERVER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_conf.h"

#ifdef USBPD_LED_SERVER
#define LED_BSP_TypeDef USBPD_BSP_Led_TypeDef

/* Exported types ------------------------------------------------------------*/
 typedef enum {
    LED_MODE_INVALID         = 0xFF,
    LED_MODE_OFF             = 0,
    LED_MODE_ON              = 1,
    LED_MODE_BLINK           = 2,
    LED_MODE_BLINK_MAP       = 3, //Map blink
    LED_MODE_BLINK_ROLE_SRC  = 4, //Role - 1 blink
    LED_MODE_BLINK_ROLE_SNK  = 5, //Role - 2 blinks
    LED_MODE_BLINK_ROLE_DRP  = 6, //Role - under definition blink
    LED_MODE_BLINK_VBUS      = 7, //Vbus - 1 blink
    LED_MODE_BLINK_CC1       = 8, //CC1 - 1 blink
    LED_MODE_BLINK_CC2       = 9, //CC2 - 2 blinks
 } LED_Mode;
 
 typedef struct {
   LED_BSP_TypeDef CCLine;
   LED_BSP_TypeDef VBus;
   LED_BSP_TypeDef Role;
 } LED_Roles;
 
/* Exported constants --------------------------------------------------------*/
#define LED_MODE_BLINK_CC(CCx) (((CCx) == (CC1)) ? (LED_MODE_BLINK_CC1) : (LED_MODE_BLINK_CC2))
 
#define LED_OFF                 0
#define LED_ON                  (!LED_OFF)

#define LED_INDEX_LEN            USBPD_BSP_LED_LEN
#define LED_INDEX_INVALID        0xFF
#define LED_INDEX_DEFAULT        0
#define LED_PERIOD_INVALID       0xFF 
#define LED_PERIOD_MIN           10      /* ms (0.01 sec */
#define LED_PERIOD_MAX           10000   /* ms (10 sec) */
#define LED_PERIOD_DEFAULT       100     /* ms (0.1 sec) */

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define LED_INDEX_IsValid(val) (((val) >= 0) && ((val) < (LED_INDEX_LEN)))
#define LED_MODE_DEFAULT LED_MODE_OFF
#define LED_MODE_IsValid(mode) (((mode) == (LED_MODE_OFF)) || ((mode) == (LED_MODE_ON)) || (((mode) >= (LED_MODE_BLINK))  && ((mode) <= (LED_MODE_BLINK_CC2))) )
#define LED_MODE_IsBlinking(mode)  ( ((mode) >= LED_MODE_BLINK_MAP) && ((mode) <= (LED_MODE_BLINK_CC2)) )
#define LED_MODE_FromPowerRole(powerrole) ( \
   ((powerrole) == USBPD_PORTPOWERROLE_DRP_SRC || (powerrole) == USBPD_PORTPOWERROLE_DRP_SNK) ? \
      LED_MODE_BLINK_ROLE_DRP : \
      (((powerrole) == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC) \
   )
#define LED_PERIOD_IsValid(mode, val) ( ((mode) != LED_MODE_BLINK) || ( ((val) >= (LED_PERIOD_MIN)) && ((val) <= (LED_PERIOD_MAX)) ) )
/* Exported functions ------------------------------------------------------- */ 

/** @addtogroup Led_Server_Main
  * @{
  */
/* CLI Command LED service interaction ****************************************/
void Led_Init(void);
void Led_Set(LED_BSP_TypeDef Index, LED_Mode Mode, uint16_t Period);
void Led_OrderedSetBits(uint8_t LedBitmap);

/**
  * @}
  */
#endif /* USBPD_LED_SERVER */

#ifdef __cplusplus
}
#endif

#endif /*__LED_SERVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
