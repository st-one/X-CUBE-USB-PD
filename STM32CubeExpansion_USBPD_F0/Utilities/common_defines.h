/**
  ******************************************************************************
  * @file    common_defines.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   This file contains the headers of common_defines.h
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
#ifndef __COMMON_DEFINES_H_
#define __COMMON_DEFINES_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
 /**
  * @brief This struct contains parameter used to define array of pins
  */
 typedef struct
 {
	GPIO_TypeDef* 	GPIOx;			/**<The GPIO Port of the Pin */
	uint16_t      	GPIO_Pin;		/**<The GPIO_Pin */
	uint32_t		ADCCH;			/**<The ADC Channel if used */
 } USBPDM1_GPIOPins_TypeDef;
 
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define MV2ADC(__X__)           ( (__X__*4095) / 3300 )
#define ADC2MV(__X__)           ( (__X__*3300) / 4095 )
 
#define USBPDM1PIN(PORT,PIN)  	{ PORT, GPIO_PIN_ ## PIN, 0 }
#define USBPDM1ADC(PORT,PIN,CH)	{ PORT, GPIO_PIN_ ## PIN, CH }

/* Stringify macros */
#define _STR(x)         #x
#define STR(x)          _STR(x)

 /* Macros for integer division with various rounding variants default integer 
    division rounds down. */
#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* Exported functions ------------------------------------------------------- */ 


#ifdef __cplusplus
}
#endif

#endif /*__COMMON_DEFINES_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
