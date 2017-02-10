/**
  ******************************************************************************
  * @file    p-nucleo-usb001.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file provides set of functions to manage peripherals on
  *          P-NUCLEO-USB001 board.
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
#include "p-nucleo-usb001.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup P-NUCLEO-USB001
  * @{
  */   

/** @defgroup P-NUCLEO-USB001_Private_Defines Private Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup P-NUCLEO-USB001_Private_Variables Private Variables
  * @{
  */ 

/**
  * @brief Vector storing informations on pins controlling leds of P-NUCLEO-USB001
  */
USBPD_BSP_GPIOPins_TypeDef USBPD_BSP_LEDs[USBPD_BSP_LEDn] =
{
  USBPD_BSP_PIN(GPIOC,10),         /* Blue,   Port 1 - Role */
  USBPD_BSP_PIN(GPIOC,11),         /* Green,  Port 1 - Vbus */
#ifndef P_NUCLEO_USB001_USE_I2C
  USBPD_BSP_PIN(GPIOB,11),         /* Orange, Port 1 - CC */
  USBPD_BSP_PIN(GPIOB,10),         /* Blue,   Port 0 - Role */
#endif /*P_NUCLEO_USB001_USE_I2C*/
#ifndef P_NUCLEO_USB001_GPIO13
  USBPD_BSP_PIN(GPIOC,15),         /* Green,  Port 0 - Vbus */
#endif /*P_NUCLEO_USB001_GPIO13*/
#ifndef P_NUCLEO_USB001_GPIO15
  USBPD_BSP_PIN(GPIOF,1),          /* Orange, Port 0 - CC */
#endif /*P_NUCLEO_USB001_GPIO15*/
};

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configures P-NUCLEO-USB001 LED GPIO.
  * @param  None
  * @retval None
  */
void USBPD_BSP_LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t led=0;
  
  /* Common values for Leds GPIO */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  for(led=0;led<USBPD_BSP_LEDn;led++)
  {
    /* Configure the GPIO pin */
    GPIO_InitStruct.Pin = USBPD_BSP_LEDs[led].GPIO_Pin;
    
    /* Init the associated GPIO */
    HAL_GPIO_Init(USBPD_BSP_LEDs[led].GPIOx, &GPIO_InitStruct);
    /* Turn the led off */
    USBPD_BSP_LED_Off((USBPD_BSP_Led_TypeDef)led);
  }
}

/**
  * @brief  Turns selected LED On or Off.
  * @param  Led: Specifies the Led to be set on.
  * @param  Value: value to set the led on or off.
  * @retval None
  */
void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, Value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */
void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */
void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  * @retval None
  */
void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin);
}

/**
  * @brief  Configures the UART used by the P-NUCLEO-USB001
  * @retval None
  */
void USBPD_BSP_UART_Init(void)
{
  /* Init huart_usbpd */
  USBPD_UART_IO_Init();
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
