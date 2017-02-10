/**
  ******************************************************************************
  * @file    p-nucleo-usb001.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains the headers of p-nucleo-usb001.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __P_NUCLEO_USB001_H_
#define __P_NUCLEO_USB001_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @addtogroup BSP
  * @{
  * */

/**
  * @addtogroup P-NUCLEO-USB001 P-NUCLEO-USB001
  * @{
  * */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_conf.h"

/** @defgroup P-NUCLEO-USB001_Exported_Macros Exported Macros
  * @{
  */
#define USBPD_BSP_PIN(PORT,PIN)    { PORT, GPIO_PIN_ ## PIN, 0 }
#define USBPD_BSP_ADC(PORT,PIN,CH) { PORT, GPIO_PIN_ ## PIN, CH }

/**
  * @}
  */

/** @defgroup P-NUCLEO-USB001_Exported_Types Exported Types
  * @{
  */

/**
  * @brief This struct contains parameter used to define array of pins
  */
typedef struct
{
      GPIO_TypeDef*   GPIOx;      /*!< The GPIO Port of the Pin */
      uint16_t        GPIO_Pin;   /*!< The GPIO_Pin */
      uint32_t        ADCCH;      /*!< The ADC Channel if used */
} USBPD_BSP_GPIOPins_TypeDef;

/**
  * @brief Leds onboard P-NUCLEO-USB001
  */
typedef enum
{
  ULED_NA = -1,
  ULED0 = 0,                         /*< LED0 (Blue); D200 of P-NUCLEO-USB001                */
  ULED1 = 1,                         /*< LED1 (Green); D201 of P-NUCLEO-USB001               */

#ifndef P_NUCLEO_USB001_USE_I2C
  ULED2 = 2,                         /*< LED2 (Orange); D202 of P-NUCLEO-USB001              */
  ULED3 = 3,                         /*< LED3 (Red); D203 of P-NUCLEO-USB001                 */
  ULED_RED = ULED3,                  /*< Redefinition of LED2 (Red); D202 of P-NUCLEO-USB001 */
#endif /*P_NUCLEO_USB001_USE_I2C*/

#ifndef P_NUCLEO_USB001_GPIO13
  ULED4 = 4,                         /*< LED4 (Orange); D204 of P-NUCLEO-USB001              */
#endif /*P_NUCLEO_USB001_GPIO13*/

#ifndef P_NUCLEO_USB001_GPIO15
  ULED5 = 5,                         /*< LED5 (Orange); D205 of P-NUCLEO-USB001               */
#endif /*P_NUCLEO_USB001_GPIO15*/

  ULED_BLUE = ULED0,                 /*< Redefinition of LED0 (Blue); D200 of P-NUCLEO-USB001  */
  ULED_GREEN = ULED1,                /*< Redefinition of LED1 (Green); D201 of P-NUCLEO-USB001 */

  /* BSP name list */
  LED_PORT0_CC    = ULED5,
  LED_PORT0_VBUS  = ULED4,
  LED_PORT0_ROLE  = ULED3,
  LED_PORT1_CC    = ULED2,
  LED_PORT1_VBUS  = ULED1,
  LED_PORT1_ROLE  = ULED0,
} USBPD_BSP_Led_TypeDef;

/**
  * @}
  */ 

/** @defgroup P-NUCLEO-USB001_Exported_Constants Exported Constants
  * @{
  */

/**
  * @briP-NUCLEO-USB001ef Number of Leds on P-NUCLEO-USB001
  * @note  This defines should come from usbpd_conf.h
  */
#ifdef P_NUCLEO_USB001_USE_I2C
#if defined(P_NUCLEO_USB001_GPIO13) && defined(P_NUCLEO_USB001_GPIO15)
  #define USBPD_BSP_LEDn  2
#elif defined(P-NUCLEO-USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPD_BSP_LEDn  3
#else
  #define USBPD_BSP_LEDn  4
#endif
#else /*P_NUCLEO_USB001_USE_I2C not defined*/
#if defined(P_NUCLEO_USB001_GPIO13) && defined(P_NUCLEO_USB001_GPIO15)
  #define USBPD_BSP_LEDn  4
#elif defined(P_NUCLEO_USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPD_BSP_LEDn  5
#else
  #define USBPD_BSP_LEDn  6
#endif
#endif /*P_NUCLEO_USB001_USE_I2C */

#define USBPD_BSP_LED_LEN USBPD_BSP_LEDn
/**
  * @}
  */ 

/** @defgroup P-NUCLEO-USB001_Exported_Functions Exported Functions
* @{
*/

void USBPD_BSP_LED_Init(void);
void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value);
void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led);
void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led);
void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led);
void USBPD_BSP_UART_Init(void);

/* UART IO functions */
void     USBPD_UART_IO_Init(void);
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

#endif /* __P_NUCLEO_USB001_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
