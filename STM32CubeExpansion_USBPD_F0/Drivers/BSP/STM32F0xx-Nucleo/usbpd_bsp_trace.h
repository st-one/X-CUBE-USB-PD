/**
  ******************************************************************************
  * @file    usbpd_bsp_trace.h
  * @author  MCD Application Team
  * @brief   This file contains bsp interface control functions.
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

#ifndef USBPD_BSP_TRACE_H
#define USBPD_BSP_TRACE_H
/* Includes ------------------------------------------------------------------*/

/* Exported defines -----------------------------------------------------------*/
#if defined(USBPD_TCPM_MODULE_ENABLED)
#define TRACE_TX_DMA_IRQ_HAND   DMA1_Channel4_5_6_7_IRQHandler
#define TRACE_USART_IRQ_HAND    USART3_4_IRQHandler
#else
#define TRACE_TX_DMA_IRQ_HAND   DMA1_Channel2_3_IRQHandler
#define TRACE_USART_IRQ_HAND    USART1_IRQHandler
#endif /* USBPD_TCPM_MODULE_ENABLED */
/* Exported functions ---------------------------------------------------------*/

void                    BSP_TRACE_Init(void (*callbackTX)(void), void (*callbackRX)(uint8_t, uint8_t));
void                    BSP_TRACE_DeInit(void);
void                    BSP_TRACE_RegisterRxCallback(void (*callbackRX)(uint8_t, uint8_t));
void                    BSP_TRACE_IRQHandlerDMA(void);
void                    BSP_TRACE_StartRX(void);
void                    BSP_TRACE_IRQHandlerUSART(void);
void                    BSP_TRACE_SendData(uint8_t *data, uint32_t size);

#endif /* USBPD_BSP_TRACE_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

