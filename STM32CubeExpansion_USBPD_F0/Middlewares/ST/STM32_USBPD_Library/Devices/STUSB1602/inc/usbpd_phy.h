/**
  ******************************************************************************
  * @file    usbpd_phy.h
  * @author  System Lab - Sensing & Connectivity Application Team
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

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PHY
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Exported_TypeDef USBPD DEVICE PHY Exported TypeDef
  * @brief   The PHY export the callbacks to receive feedbacks from the USBPD_CORE_PRL
  * @{
  */

/**
  * @brief CallBacks exposed by the @ref USBPD_DEVICE_PHY to the USBPD_CORE_PRL
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
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_PHY_Exported_Functions
  * @brief API provided by the PHY for the upper layer
  * @details first time the entity have to initizialize the layer calling @ref USBPD_PHY_Init function<br/>
  *      <ul>
  *       <li>@ref USBPD_PHY_SendMessage and @ref USBPD_PHY_ResetRequest send a message on the communication channel</li>
  *       <li>The incoming message are received by callback and the PHY notifies to upper layer by another callback</li>
  *      </ul>
  * @{
  */
USBPD_StatusTypeDef USBPD_PHY_Init(uint8_t PortNum, USBPD_PHY_Callbacks *cbs, uint8_t *pRxBuffer, USBPD_PortPowerRole_TypeDef role, uint32_t SupportedSOP);
void                USBPD_PHY_Reset(uint8_t PortNum);
uint32_t            USBPD_PHY_GetRetryTimerValue(uint8_t PortNum);
void                USBPD_PHY_EnableRX(uint8_t PortNum);
void                USBPD_PHY_DisableRX(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type);
USBPD_StatusTypeDef USBPD_PHY_SendMessage(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint16_t Size);
USBPD_StatusTypeDef USBPD_PHY_Send_BIST_Pattern(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_PHY_ExitTransmit(uint8_t PortNum, USBPD_SOPType_TypeDef BistType);
void                USBPD_PHY_SetResistor_SinkTxNG(uint8_t PortNum);
void                USBPD_PHY_SetResistor_SinkTxOK(uint8_t PortNum);
uint8_t             USBPD_PHY_IsResistor_SinkTxOk(uint8_t PortNum);
void                USBPD_PHY_FastRoleSwapSignalling(uint8_t PortNum);
void                USBPD_PHY_SOPSupported(uint8_t PortNum,uint32_t SOPSupported);

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
