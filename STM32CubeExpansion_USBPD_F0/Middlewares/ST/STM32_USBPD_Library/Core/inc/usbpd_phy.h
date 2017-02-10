/**
  ******************************************************************************
  * @file    usbpd_phy.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains the headers of usbpd_phy.h.
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

#ifndef __USBPD_PHY_H_
#define __USBPD_PHY_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
//#include "usbpd_conf.h"
#include "usbpd_hw_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_PHY
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_CORE_PHY_Exported_TypeDef USBPD CORE PHY Exported TypeDef
  * @{
  */
/**
  * @brief CallBacks exposed by the @ref USBPD_CORE_PHY to the @ref USBPD_CORE_PRL
  */
typedef struct
{
  /**
   * @brief  Reports that a message has been received on a specified port.
   * @param  PortNum:    The handle of the port
   * @param  Type:    The type of the message received
   * @retval None
   * @note Received data are stored inside PortNum->pRxBuffPtr
   */
  void (*USBPD_PHY_MessageReceived)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
   * @brief  Reports to the PRL that a Reset received from channel.
   * @param  PortNum:    The handle of the port
   * @param  Type:    The type of reset performed
   * @retval None
   */
  void (*USBPD_PHY_ResetIndication)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
   * @brief  Reports to the PRL that a Reset operation has been completed.
   * @param  PortNum:    The handle of the port
   * @param  Type:    The type of reset performed
   * @retval None
   */
  void (*USBPD_PHY_ResetCompleted)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
   * @brief  Reports the PHY layer discarded last message to sent because the bus is not idle.
   * @param  PortNum:    The handle of the port
   * @retval None
   * @note See Section 5.7 of USB Power Delivery specification Rev2, V1.1
   */
  void (*USBPD_PHY_ChannelIdleAfterBusy)(uint8_t PortNum);

  /**
   * @brief  Reports to the PRL that a Bist operation has been completed.
   * @param  PortNum:    The handle of the port
   * @param  Type:    The type of Bist performed
   * @retval None
   */
  void (*USBPD_PHY_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);
        
 /**
   * @brief  Reports to the PRL that a tx operation has been completed.
   * @param  PortNum:    The handle of the port
   * @retval None
   */
  void (*USBPD_PHY_TxCompleted)(uint8_t PortNum);
        
} USBPD_PHY_Callbacks;

 /**
  * @brief Initialization structure exposed by the @ref USBPD_CORE_PHY to the @ref USBPD_CORE_PRL
  * */
typedef struct
{
  USBPD_PHY_Callbacks cbs; /**< Callbacks */
} USBPD_PHY_HandleTypeDef;

/** 
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_CORE_PHY_Exported_Functions
  * @{
  */
USBPD_StatusTypeDef USBPD_PHY_Init(uint8_t PortNum, USBPD_PHY_Callbacks cbs, uint8_t *pRxBuffer, USBPD_PortPowerRole_TypeDef role);
void                USBPD_PHY_Reset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type);
USBPD_StatusTypeDef USBPD_PHY_SendMessage(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint8_t Size);
USBPD_StatusTypeDef USBPD_PHY_Send_BIST_Pattern(uint8_t PortNum);

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

#endif /* __USBPD_PHY_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
