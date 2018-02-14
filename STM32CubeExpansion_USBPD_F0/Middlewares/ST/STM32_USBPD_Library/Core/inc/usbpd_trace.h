/**
  ******************************************************************************
  * @file    usbpd_trace.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_cad.h for Cable Attach-Detach
  *          controls.
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
#ifndef __USBPD_TRACE_H_
#define __USBPD_TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(_GUI_INTERFACE)
#include "GUI_API.h"
#endif /* _GUI_INTERFACE */

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_TRACE
  * @{
  */


/* Exported types ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_TRACE_Exported_Types USBPD CORE TRACE Exported Types
  * @{
  */
typedef enum {
  USBPD_TRACE_GUI         = 0,
  USBPD_TRACE_MESSAGE_IN  = 1,
  USBPD_TRACE_MESSAGE_OUT,
  USBPD_TRACE_CADEVENT,
  USBPD_TRACE_PE_STATE,
  USBPD_TRACE_CAD_LOW
}
TRACE_EVENT;

/**
  * @}
  */
/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/** @defgroup USBPD_CORE_TRACE_Exported_Defines USBPD CORE TRACE Exported Defines
  * @{
  */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_CORE_CAD_Exported_Functions
  * @{
  */
void            USBPD_TRACE_Init(void);
void            USBPD_TRACE_Add(TRACE_EVENT type, uint8_t port, uint8_t sop, uint8_t *ptr, uint32_t size);
uint32_t        USBPD_TRACE_TX_Process(void);
#if defined(_GUI_INTERFACE)
uint32_t        USBPD_TRACE_RX_Process(void);
uint32_t        USBPD_TRACE_SendNotification(uint32_t PortNum, uint32_t TypeNotification, uint32_t Value);
#endif /* _GUI_INTERFACE */



/**
  * @}
  */

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

#endif /* __USBPD_CAD_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

