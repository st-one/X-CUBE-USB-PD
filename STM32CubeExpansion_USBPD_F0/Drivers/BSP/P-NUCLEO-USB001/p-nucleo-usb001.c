/**
  ******************************************************************************
  * @file    p-nucleo-usb001.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   This file provides set of functions to manage peripherals on
  *          P-NUCLEO-USB001 board.
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
  * @brief Usart BaudRate used by P-NUCLEO-USB001
  */
#define BAUDRATE                             115200

/**
  * @brief Usart used by P-NUCLEO-USB001
  */
#define P_NUCLEO_USB001_USART               USART1

/**
  * @brief Clock Enable Macro
  */
#define P_NUCLEO_USB001_USARTCLK_ENABLE     __USART1_CLK_ENABLE

/**
  * @brief Usart Pin
  */
#define USART_TX_PORT                        GPIOA
#define USART_TX_PIN                         GPIO_PIN_9
#define USART_RX_PORT                        GPIOA
#define USART_RX_PIN                         GPIO_PIN_10
#define USART_PIN_GPIOAF                     GPIO_AF1_USART1
#define USART_IRQ                            USART1_IRQn

/**
  * @}
  */

/** @defgroup P-NUCLEO-USB001_Private_Variables Private Variables
  * @{
  */ 

/**
  * @brief Vector storing informations on pins controlling leds of P-NUCLEO-USB001
  */
USBPDM1_GPIOPins_TypeDef USBPDM1_LEDs[USBPDM1_LEDn] =
{
  USBPDM1PIN(GPIOC,10),         /* Blue,   Port 1 - Role */
  USBPDM1PIN(GPIOC,11),         /* Green,  Port 1 - Vbus */
#ifndef P_NUCLEO_USB001_USE_I2C
  USBPDM1PIN(GPIOB,11),         /* Orange, Port 1 - CC */
  USBPDM1PIN(GPIOB,10),         /* Blue,   Port 0 - Role */
#endif /*P_NUCLEO_USB001_USE_I2C*/
#ifndef P_NUCLEO_USB001_GPIO13
  USBPDM1PIN(GPIOC,15),         /* Green,  Port 0 - Vbus */
#endif /*P_NUCLEO_USB001_GPIO13*/
#ifndef P_NUCLEO_USB001_GPIO15
  USBPDM1PIN(GPIOF,1),          /* Orange, Port 0 - CC */
#endif /*P_NUCLEO_USB001_GPIO15*/
};

/**
  * @brief huart handle for P-NUCLEO-USB001
  */
UART_HandleTypeDef huart_usbpdm1;

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
void USBPDM1_LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t led=0;
  
  /* Common values for Leds GPIO */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  
  for(led=0;led<USBPDM1_LEDn;led++)
  {
    /* Configure the GPIO pin */
    GPIO_InitStruct.Pin = USBPDM1_LEDs[led].GPIO_Pin;
    
    /* Init the associated GPIO */
    HAL_GPIO_Init(USBPDM1_LEDs[led].GPIOx, &GPIO_InitStruct);
    /* Turn the led off */
    USBPDM1_LED_Off((USBPDM1_Led_TypeDef)led);
  }
}

/**
  * @brief  Turns selected LED On or Off.
  * @param  Led: Specifies the Led to be set on.
  * @param  Value: value to set the led on or off.
  * @retval None
  */
void USBPDM1_LED_Set(USBPDM1_Led_TypeDef Led, uint8_t Value)
{
  HAL_GPIO_WritePin(USBPDM1_LEDs[Led].GPIOx, USBPDM1_LEDs[Led].GPIO_Pin, Value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */
void USBPDM1_LED_On(USBPDM1_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPDM1_LEDs[Led].GPIOx, USBPDM1_LEDs[Led].GPIO_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */
void USBPDM1_LED_Off(USBPDM1_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPDM1_LEDs[Led].GPIOx, USBPDM1_LEDs[Led].GPIO_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  * @retval None
  */
void USBPDM1_LED_Toggle(USBPDM1_Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(USBPDM1_LEDs[Led].GPIOx, USBPDM1_LEDs[Led].GPIO_Pin);
}

/**
  * @brief  Configures the UART used by the P-NUCLEO-USB001
  * @retval None
  */
void USBPDM1_UART_Init(void)
{
  /* Init huart_usbpdm1 */
  huart_usbpdm1.Instance = P_NUCLEO_USB001_USART;
  huart_usbpdm1.Init.BaudRate = BAUDRATE;
  huart_usbpdm1.Init.WordLength = UART_WORDLENGTH_8B;
  huart_usbpdm1.Init.StopBits = UART_STOPBITS_1;
  huart_usbpdm1.Init.Parity = UART_PARITY_NONE;
  huart_usbpdm1.Init.Mode = UART_MODE_TX_RX;
  huart_usbpdm1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_usbpdm1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart_usbpdm1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart_usbpdm1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  /* Init of the peripheral */
  HAL_UART_Init(&huart_usbpdm1);
}

/**
  * @brief  Configures the UART low level hardware : GPIO, CLOCK.
  * @param  huart: UAART handle
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Peripheral clock enable */
  P_NUCLEO_USB001_USARTCLK_ENABLE();
  
  /* USART GPIO Configuration */
  GPIO_InitStruct.Pin = USART_TX_PIN | USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USART_PIN_GPIOAF;
  HAL_GPIO_Init(USART_TX_PORT, &GPIO_InitStruct);
  
  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(USART_IRQ, 3, 0);
  HAL_NVIC_EnableIRQ(USART_IRQ);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
