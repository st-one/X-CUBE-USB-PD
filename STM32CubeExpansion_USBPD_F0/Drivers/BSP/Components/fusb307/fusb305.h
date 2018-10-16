/**
  ******************************************************************************
  * @file    fusb305.h
  * @author  MCD Application Team
  * @version $VERSION$
  * @date    $DATE$
  * @brief   This file contains all the functions prototypes for the TCPC driver.   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUSB305_H
#define __FUSB305_H


#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "usbpd_core.h"
 
/** @addtogroup TCPC_Driver
  * @{
  */

/** @addtogroup FUSB305_TCPC
  * @{
  */

/** @addtogroup FUSB305_TCPC_Private_Defines
 * @{
 */
#if !defined(__PACKED)
    #define __PACKED
#endif

/* Chip Device ID - 302A or 302B */
#define FUSB305_DEVID_302A 0x08
#define FUSB305_DEVID_302B 0x09

/* I2C slave address varies by part number */
/* FUSB305BUCX / FUSB305BMPX */
#define FUSB305_I2C_SLAVE_ADDR      0x44
/* FUSB305B01MPX */
#define FUSB305_I2C_SLAVE_ADDR_B01  0x46
/* FUSB305B10MPX */
#define FUSB305_I2C_SLAVE_ADDR_B10  0x48
/* FUSB305B11MPX */
#define FUSB305_I2C_SLAVE_ADDR_B11  0x4A

/* Default retry count for transmitting */
#define PD_RETRY_COUNT      3

/**
  * @}
  */

/** @defgroup FUSB305_TCPC_Exported_TypeDef FUSB305 Exported TypeDef
  * @{
  */

typedef enum {
  TCPC_REG_VCONN_OCP                      = 0xA0u,
  TCPC_REG_RESET                          = 0xA2u,
  TCPC_REG_GPIO1_CFG                      = 0xA4u,
  TCPC_REG_GPIO2_CFG                      = 0xA5u,
  TCPC_REG_GPIO_STAT                      = 0xA6u,
  TCPC_REG_DRPTOGGLE                      = 0xA7u,
  TCPC_REG_TOGGLE_SM                      = 0xA8u,
  TCPC_REG_SNK_TRANSMIT                   = 0xB0u,
  TCPC_REG_SRC_FRSWAP                     = 0xB1u,
  TCPC_REG_SNK_FRSWAP                     = 0xB2u,
  TCPC_REG_ALERT_VD                       = 0xB3u,
  TCPC_REG_ALERT_VD_MASK                  = 0xB4u,
} USBPD_TCPC_VendorRegisterId;

/* TCPC driver structure */
extern TCPC_DrvTypeDef   fusb305_tcpc_drv;
/**
  * @}
  */

/** @addtogroup FUSB305_TCPC_Exported_Functions
  * @{
  */
USBPD_StatusTypeDef fusb305_tcpc_init(uint32_t PortNum, USBPD_PortPowerRole_TypeDef Role, uint8_t ToggleRole, uint8_t (*IsSwapOngoing)(uint8_t));
USBPD_StatusTypeDef fusb305_tcpc_get_cc(uint32_t PortNum, uint32_t *cc1, uint32_t *cc2);
USBPD_StatusTypeDef fusb305_tcpc_get_vbus_level(uint32_t PortNum, uint8_t *VBUSLevel, uint16_t *VBUSVoltage);
USBPD_StatusTypeDef fusb305_tcpc_set_vbus_level(uint32_t PortNum, USBPD_FunctionalState State);
USBPD_StatusTypeDef fusb305_tcpc_get_power_status(uint32_t PortNum, uint8_t *PowerStatus);
USBPD_StatusTypeDef fusb305_tcpc_get_fault_status(uint32_t PortNum, uint8_t *FaultStatus);
USBPD_StatusTypeDef fusb305_tcpc_set_fault_status(uint32_t Port, uint8_t FaultStatus);
USBPD_StatusTypeDef fusb305_tcpc_set_cc(uint32_t PortNum, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State);
USBPD_StatusTypeDef fusb305_tcpc_set_polarity(uint32_t PortNum, uint8_t Polarity);
USBPD_StatusTypeDef fusb305_tcpc_set_vconn(uint32_t PortNum, USBPD_FunctionalState State);
USBPD_StatusTypeDef fusb305_tcpc_set_msg_header(uint32_t PortNum, USBPD_PortPowerRole_TypeDef PowerRole, USBPD_PortDataRole_TypeDef DataRole);
USBPD_StatusTypeDef fusb305_tcpc_alert_status(uint32_t PortNum, uint16_t *Alert);
USBPD_StatusTypeDef fusb305_tcpc_set_rx_state(uint32_t Port, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State, uint32_t SupportedSOP, TCPC_hard_reset HardReset);
USBPD_StatusTypeDef fusb305_tcpc_set_sop_supported(uint32_t Port, uint32_t SupportedSOP);
USBPD_StatusTypeDef fusb305_tcpc_set_power_status_mask(uint32_t PortNum, uint8_t Mask);
USBPD_StatusTypeDef fusb305_tcpc_alert_mask_set(uint32_t PortNum, uint16_t Mask);
USBPD_StatusTypeDef fusb305_tcpc_get_message(uint32_t PortNum, uint8_t *Payload, uint8_t *Type);
USBPD_StatusTypeDef fusb305_tcpc_transmit(uint32_t PortNum, USBPD_SOPType_TypeDef Type, uint16_t Header, const uint8_t *pData, uint32_t RetryNumber);
USBPD_StatusTypeDef fusb305_tcpc_alert(uint32_t PortNum, uint16_t *Alert);
USBPD_StatusTypeDef fusb305_tcpc_clear_alert(uint32_t PortNum, uint16_t *Alert);
USBPD_StatusTypeDef fusb305_tcpc_set_bist_test_data(uint32_t PortNum, uint8_t Enable);
USBPD_StatusTypeDef fusb305_tcpc_SinkTxNG(uint32_t PortNum);
USBPD_StatusTypeDef fusb305_tcpc_SinkTxOK(uint32_t PortNum);
USBPD_StatusTypeDef fusb305_tcpc_IfSinkTxOk(uint32_t PortNum);
/**
  * @}
  */

/** @defgroup FUSB305_TCPC_Exported_IO_Functions FUSB305 Exported IO Functions
  * @{
  */
/* TCPC IO functions */
USBPD_StatusTypeDef USBPD_TCPCI_ReadRegister(uint8_t PortNum, uint8_t registerId, uint8_t *prtData, uint8_t datasize);
USBPD_StatusTypeDef USBPD_TCPCI_WriteRegister(uint8_t PortNum, uint8_t registerId, uint8_t *prtData, uint8_t datasize);
USBPD_StatusTypeDef USBPD_TCPCI_SendTransmitBuffer(uint32_t PortNum, uint8_t RegisterId, uint8_t TransmitByteCount, uint16_t Header, uint8_t *pData);
USBPD_StatusTypeDef USBPD_TCPCI_ReceiveBuffer(uint32_t PortNum, uint8_t registerId, uint8_t *Buffer, uint8_t *SOPType);
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

#endif /* __FUSB305_H */

