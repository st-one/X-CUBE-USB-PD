/**
  ******************************************************************************
  * @file    p-nucleo-usb002.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of p-nucleo-usb002.c file.
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

#ifndef __P_NUCLEO_USB002_H_
#define __P_NUCLEO_USB002_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @addtogroup BSP
  * @{
  * */

/**
  * @addtogroup P-NUCLEO-USB002
  * @{
  * */

/* Includes ------------------------------------------------------------------*/
#if defined(STM32F072xB) || defined(STM32F051x8)
#include "stm32f0xx.h"
#elif defined(STM32F334x8)
#include "stm32f3xx.h"
#else
#error "Add include for the family!"
#endif
#if defined(USBPD_STUSB1605)
#include "tcpc.h"
#endif /* USBPD_STUSB1605 */

/** @defgroup P-NUCLEO-USB001_Exported_Macros Exported Macros
  * @{
  */
#define USBPD_BSP_PIN(PORT,PIN)    { PORT, GPIO_PIN_ ## PIN, 0 }
#define USBPD_BSP_ADC(PORT,PIN,CH) { PORT, GPIO_PIN_ ## PIN, CH }

/**
  * @}
  */

/** @defgroup P-NUCLEO-USB002_Exported_Types Exported Types
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
  * @brief Leds on board P_NUCLEO_USB002
  *
  */ 
  typedef enum
  {
    ELED  = -1,
    ELED0 = 0,       /**<LED1 (green);  D107 of MB1303 X-NUCLEO */
    
    ELED1 = 1,       /**<LED01 (blue);  D100 of MB1303 X-NUCLEO */
    ELED2 = 2,       /**<LED02 (green); D101 of MB1303 X-NUCLEO */
    ELED3 = 3,       /**<LED03 (red);   D102 of MB1303 X-NUCLEO */
    
    ELED4 = 4,       /**<LED11 (blue);  D103 of MB1303 X-NUCLEO */
    ELED5 = 5,       /**<LED12 (green); D104 of MB1303 X-NUCLEO */
    ELED6 = 6,       /**<LED13 (red);   D105 of MB1303 X-NUCLEO */

    /* BSP name list */
    GREEN_USER_LED      = ELED0,

    LED_PORT0_CC        = ELED3,
    LED_PORT0_VBUS      = ELED2,
#if defined(__AUTHENTICATION__)
    /* In case of Authentication, LED01 and LED13 are mapped on 
       GPIOs used for I2C ST-SAFE communication. As only one port is handled,
       PORT0 Role is remapped on original PORT1 Role => ELED4 */
    LED_PORT0_ROLE      = ELED4,
#else
    LED_PORT0_ROLE      = ELED1,
#endif  /* __AUTHENTICATION__ */

    LED_PORT1_CC        = ELED6,
    LED_PORT1_VBUS      = ELED5,
    LED_PORT1_ROLE      = ELED4
  } USBPD_BSP_Led_TypeDef;

/**
  * @}
  */ 

  
/* Exported define -----------------------------------------------------------*/
/** @defgroup P-NUCLEO-USB002_Exported_Constants Exported Constants
  * @{
  */

#define USBPD_BSP_I2CxHandle                    hi2c2

#if defined(USBPD_CLI)
#define USBPD_BSP_USART_IRQHandler              USART1_IRQHandler
#endif /* USBPD_CLI */

#define USBPD_BSP_LEDn                          7
#define USBPD_BSP_LED_LEN                       USBPD_BSP_LEDn

#if defined(USBPD_STUSB1605)
#define ALERT_PORT_INDEX(__PORT__)              ((__PORT__ == 0) ? 0            : 1 )  
#define ALERT_GPIO_PORT(__PORT__)               ( GPIOA )  
#define ALERT_GPIO_PIN(__PORT__)                ((__PORT__ == 0) ? GPIO_PIN_1   : GPIO_PIN_2 )
#define ALERT_GPIO_IRQHANDLER(__PORT__)         ((__PORT__ == 0) ? EXTI0_1_IRQn : EXTI2_3_IRQn )
#define ALERT_GPIO_IRQPRIORITY(__PORT__)        ( USBPD_LOW_IRQ_PRIO )
  
#endif /* USBPD_STUSB1605 */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
    
/* Exported variables --------------------------------------------------------*/

#if defined(USBPD_CLI)
/* UART HANDLE */
extern UART_HandleTypeDef                     huart_USBPD_BSP;
#endif /* USBPD_CLI */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup P-NUCLEO-USB002_Exported_Functions
  * @{
  */

    /**
      * @brief  Configures P_NUCLEO_USB002 LED GPIO.
      * @param  None
      * @retval None
      */
    void USBPD_BSP_LED_Init(void);

    /**
      * @brief  Turns selected LED On.
      * @param  Led: Specifies the Led to be set on.
      * @retval None
      */
    void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led);

    /**
      * @brief  Turns selected LED On or Off.
      * @param  Led: Specifies the Led to be set on.
      * @param  Value: value to set the led on or off.
      * @retval None
      */
    void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value);    
    
    /**
      * @brief  Turns selected LED Off.
      * @param  Led: Specifies the Led to be set off.
      * @retval None
      */
    void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led);

    /**
      * @brief  Toggles the selected LED.
      * @param  Led: Specifies the Led to be toggled.
      * @retval None
      */
    void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led);

#if defined(HAL_UART_MODULE_ENABLED)
    /**
      * @brief  Configures the UART used by the P-NUCLEO-USB002
      * @retval None
      */
    void USBPD_BSP_UART_Init(void);    
#endif /* HAL_UART_MODULE_ENABLED */

#if defined(USBPD_STUSB1605)
void                    USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState);
void                    USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio);
void                    USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio);
void                    USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio);

void                    BSP_USBPD_SetVoltage(uint32_t Port, uint32_t Voltage);
uint16_t                BSP_USBPD_GetVoltage(uint32_t Port);
uint16_t                BSP_USBPD_GetCurrent(uint32_t Port);

USBPD_StatusTypeDef     USBPD_TCPCI_GetDevicesDrivers(uint8_t PortNum, TCPC_DrvTypeDef **TCPC_Driver);
#endif /* USBPD_STUSB1605 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __P_NUCLEO_USB002_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
