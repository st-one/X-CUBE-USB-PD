/**
  ******************************************************************************
  * @file    usbpd_conf.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains general configuration
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
#ifndef __USBPD_CONF_H_
#define __USBPD_CONF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* including the platform */   
#include "p-nucleo-usb001.h"

/* Exported typedef ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/**
 * @brief This should be defined to use ECT I2C
 * */
#undef P_NUCLEO_USB001_USE_I2C

/**
* @brief This should be defined to use GPIO13 on PC12
* */
#undef P_NUCLEO_USB001_GPIO13

 /**
 * @brief This should be defined to use GPIO15 on PC12
 * */
#undef P_NUCLEO_USB001_GPIO15

 /**
 * @brief This should be defined to use USB 2 peripheral
 * */
#undef P_NUCLEO_USB001_USE_USB2

/**
* @brief The number of USBPD ports
* */
#define USBPD_PORT_COUNT        1  /*!< Number of supported ports */
#define USBPD_USED_PORT         0  /* uncomment this define to use Port0 */
//#define USBPD_SOURCE_ADV_1_5A_5V

/* Uncomment this define to allow sending Ping message when an Explicit Contract is established */
//#define USBPD_SEND_PING_MSG

/* Uncomment this define to allow Hard Reset if SRC_Capabilities are sent PE_NCAPSCOUNT times without answer */
//#define USBPD_PE_NCAPSCOUNT_HR

/* Uncomment this define to use the led server functionalities */
#define USBPD_LED_SERVER

/* Uncomment this define to use the CLI functionality */
#define USBPD_CLI

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* USB PD memory management macros */
#define USBPD_malloc    malloc
#define USBPD_free      free
#define USBPD_memset    memset
#define USBPD_memcpy    memcpy

/* USB PD DEBUG macros */
#if (USBPD_DEBUG_LEVEL > 0)
#define USBPD_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_UsrLog(...)
#endif 


#if (USBPD_DEBUG_LEVEL > 1)
#define USBPD_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_ErrLog(...)
#endif 


#if (USBPD_DEBUG_LEVEL > 2)
#define USBPD_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_DbgLog(...)
#endif


/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_CONF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
