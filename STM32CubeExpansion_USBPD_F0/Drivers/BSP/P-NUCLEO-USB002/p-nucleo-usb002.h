/**
  ******************************************************************************
  * @file    p-nucleo-usb002.h
  * @author  System Lab
  * @version V1.2.1
  * @date    24-Apr-2017
  * @brief   This file contains the headers of P_NUCLEO_USB002.c.
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

#ifndef __P_NUCLEO_USB002_H_
#define __P_NUCLEO_USB002_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
  
#include "usbpd_def.h"
#include "usbpd_conf.h"

  
  
/**
  * @addtogroup P-NUCLEO-USB002
  * @{
  * */

/** @defgroup P-NUCLEO-USB001_Exported_Macros Exported Macros
  * @{
  */
#define USBPD_BSP_PIN(PORT,PIN)    { PORT, GPIO_PIN_ ## PIN, 0 }
#define USBPD_BSP_ADC(PORT,PIN,CH) { PORT, GPIO_PIN_ ## PIN, CH }

/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/
  
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
 * */ 
  typedef enum
  {
    ELED  = -1,
    ELED0 = 0,       /**<LED0 (green); D107 of MB1303 X-NUCLEO */
    
    ELED1 = 1,       /**<LED1 (color); LD1 of MB1303 X-NUCLEO */
    ELED2 = 2,       /**<LED2 (color); LD2 of MB1303 X-NUCLEO */
    ELED3 = 3,       /**<LED3 (color); LD5 of MB1303 X-NUCLEO */
    
    ELED4 = 4,       /**<LED1 (color); LD1 of MB1303 X-NUCLEO */
    ELED5 = 5,       /**<LED2 (color); LD2 of MB1303 X-NUCLEO */
    ELED6 = 6,       /**<LED3 (color); LD5 of MB1303 X-NUCLEO */
      
//    /* BSP name list */
//    GREEN_USER_LED      = ELED0,
//    
//    LED_PORT0_CC        = ELED3,
//    LED_PORT0_VBUS      = ELED2,
//    LED_PORT0_ROLE      = ELED1,
//    
//    LED_PORT1_CC        = ELED6,     
//    LED_PORT1_VBUS      = ELED5,
//    LED_PORT1_ROLE      = ELED4
      
    /* BSP name list */
    GREEN_USER_LED      = ELED0,
    
    LED_PORT0_CC        = ELED3,
    LED_PORT0_VBUS      = ELED2,
    LED_PORT0_ROLE      = ELED1,
    
    LED_PORT1_CC        = ELED6,     
    LED_PORT1_VBUS      = ELED5,
    LED_PORT1_ROLE      = ELED4
  } USBPD_BSP_Led_TypeDef;

  
/* Exported define -----------------------------------------------------------*/

#define USBPD_BSP_I2CxHandle                    hi2c2

#define USBPD_BSP_USART_IRQHandler              USART2_IRQHandler

#define USBPD_BSP_LEDn                          7
#define USBPD_BSP_LED_LEN                       USBPD_BSP_LEDn
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
    
/* Exported variables --------------------------------------------------------*/

/* UART HANDLE */
extern UART_HandleTypeDef                     huart_USBPD_BSP;

/* Exported functions --------------------------------------------------------*/

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

    /**
      * @brief  Configures the UART used by the P-NUCLEO-USB002
      * @retval None
      */
    void USBPD_BSP_UART_Init(void);    

    /** @} */

    
#ifdef __cplusplus
}
#endif

#endif /* __P_NUCLEO_USB002_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
