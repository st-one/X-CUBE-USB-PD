/**
  ******************************************************************************
  * @file    tcpc.h
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
#ifndef __TCPC_H
#define __TCPC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "usbpd_def.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup TCPC
  * @{
  */
 
#if !defined(__PACKED)
  #define __PACKED
#endif

/** @defgroup TCPC_Exported_Constants TCPC Exported Constants
  * @{
  */
enum tcpc_cc_voltage_status {
  TYPEC_CC_VOLT_OPEN = 0,
  TYPEC_CC_VOLT_RA = 1,
  TYPEC_CC_VOLT_RD = 2,
  TYPEC_CC_VOLT_SNK_DEF = 5,
  TYPEC_CC_VOLT_SNK_1_5 = 6,
  TYPEC_CC_VOLT_SNK_3_0 = 7,
};

typedef enum  {
  TYPEC_CC_RA = 0,
  TYPEC_CC_RP = 1,
  TYPEC_CC_RD = 2,
  TYPEC_CC_OPEN = 3,
}TCPC_CC_Pull_TypeDef;

typedef enum  {
  TYPEC_RP_VALUE_DEFAULT = 0,
  TYPEC_RP_VALUE_1P5A = 1,
  TYPEC_RP_VALUE_3P0A = 2,
}TCPC_RP_Value_TypeDef;

enum tcpm_transmit_type {
  TCPC_TX_SOP = 0,
  TCPC_TX_SOP_PRIME = 1,
  TCPC_TX_SOP_PRIME_PRIME = 2,
  TCPC_TX_SOP_DEBUG_PRIME = 3,
  TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
  TCPC_TX_HARD_RESET = 5,
  TCPC_TX_CABLE_RESET = 6,
  TCPC_TX_BIST_MODE_2 = 7
};

enum tcpc_transmit_complete {
  TCPC_TX_COMPLETE_SUCCESS =   0,
  TCPC_TX_COMPLETE_DISCARDED = 1,
  TCPC_TX_COMPLETE_FAILED =    2,
};

typedef enum {
  TCPC_HARD_RESET_NONE      = 0, /*!< No hard reset ongoing         */
  TCPC_HARD_RESET_SENT      = 1, /*!< Hard reset has been sent      */
  TCPC_HARD_RESET_RECEIVED  = 2, /*!< Hard reset has been received  */
}TCPC_hard_reset;

/** @defgroup TCPC_EC_REG_ALERT TCPC REG_ALERT defines
  * @{
  */
#define TCPC_REG_ALERT_CLEAR_ALL           0x8FFFu
#define TCPC_REG_ALERT_VENDOR_DEFINED      0x8000u
#define TCPC_REG_ALERT_SINK_DISCONNECTED   0x0800u
#define TCPC_REG_ALERT_RX_OVERFLOW         0x0400u
#define TCPC_REG_ALERT_FAULT               0x0200u
#define TCPC_REG_ALERT_VBUS_VOLTAGE_LO     0x0100u
#define TCPC_REG_ALERT_VBUS_VOLTAGE_HI     0x0080u
#define TCPC_REG_ALERT_TRANSMIT_SUCCESS    0x0040u
#define TCPC_REG_ALERT_TRANSMIT_DISCARD    0x0020u
#define TCPC_REG_ALERT_TRANSMIT_FAILED     0x0010u
#define TCPC_REG_ALERT_RECEIVE_HARDRESET   0x0008u
#define TCPC_REG_ALERT_RECEIVE_SOP         0x0004u
#define TCPC_REG_ALERT_POWER_STATUS        0x0002u
#define TCPC_REG_ALERT_CC_STATUS           0x0001u
#define TCPC_REG_ALERT_TRANSMIT_COMPLETE  (TCPC_REG_ALERT_TRANSMIT_SUCCESS | \
              TCPC_REG_ALERT_TRANSMIT_DISCARD | \
              TCPC_REG_ALERT_TRANSMIT_FAILED)
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_ALERT_MASK TCPC TCPC_EC_REG_ALERT_MASK defines
  * @{
  */
#define TCPC_REG_ALERT_MASK_ALL                 0xfffu
#define TCPC_REG_ALERT_MASK_SINK_DISCONNETED    0x800u
#define TCPC_REG_ALERT_MASK_RX_OVERFLOW         0x400u
#define TCPC_REG_ALERT_MASK_FAULT               0x200u
#define TCPC_REG_ALERT_MASK_VBUS_VOLTAGE_LO     0x100u
#define TCPC_REG_ALERT_MASK_VBUS_VOLTAGE_HI     0x080u
#define TCPC_REG_ALERT_MASK_TRANSMIT_SUCCESS    0x040u
#define TCPC_REG_ALERT_MASK_TRANSMIT_DISCARD    0x020u
#define TCPC_REG_ALERT_MASK_TRANSMIT_FAILED     0x010u
#define TCPC_REG_ALERT_MASK_RECEIVE_HARDRESET   0x008u
#define TCPC_REG_ALERT_MASK_RECEIVE_SOP         0x004u
#define TCPC_REG_ALERT_MASK_POWER_STATUS        0x002u 
#define TCPC_REG_ALERT_MASK_CC_STATUS           0x001u  
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_POWER_STATUS_MASK TCPC REG_POWER_STATUS_MASK defines
  * @{
  */
#define TCPC_REG_POWER_STATUS_MASK_ALL                0xFFu
#define TCPC_REG_POWER_STATUS_MASK_DEBUG_ACC          0x80u
#define TCPC_REG_POWER_STATUS_MASK_TCPC_INIT          0x40u
#define TCPC_REG_POWER_STATUS_MASK_SRC_HIGH_VOLT      0x20u
#define TCPC_REG_POWER_STATUS_MASK_VBUS_STAT          0x10u
#define TCPC_REG_POWER_STATUS_MASK_VBUS_DETC          0x08u
#define TCPC_REG_POWER_STATUS_MASK_VBUS_PRES          0x04u
#define TCPC_REG_POWER_STATUS_MASK_VCONN_PRES         0x02u
#define TCPC_REG_POWER_STATUS_MASK_SINK_VBUS_STATUSC  0x01u
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_FAULT_STATUS_MASK TCPC REG_FAULT_STATUS_MASK defines
  * @{
  */
#define TCPC_REG_FAULT_STATUS_ALL_REG_RESET_TO_DEF  0x80u
#define TCPC_REG_FAULT_STATUS_MASK_FVBUS_OFF        0x40u
#define TCPC_REG_FAULT_STATUS_MASK_AUTO_DISCHARGE   0x20u
#define TCPC_REG_FAULT_STATUS_MASK_FORCE_DISCHARGE  0x10u
#define TCPC_REG_FAULT_STATUS_MASK_OCP              0x08u
#define TCPC_REG_FAULT_STATUS_MASK_OVP              0x04u  
#define TCPC_REG_FAULT_STATUS_MASK_VCONN_OC         0x02u 
#define TCPC_REG_FAULT_STATUS_MASK_I2C              0x01u
/**
  * @}
  */
/** @defgroup TCPC_EC_REG_CONFIG_STANDARD_OUTPUT TCPC REG_CONFIG_STANDARD_OUTPUT defines
  * @{
  */
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_HIGH_IMPEDANCE        0x80u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_DEBUG_ACC             0x40u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_AUDIO_ACC             0x20u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_ACTIVE_CABLE          0x10u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_MUX_CONTROL_NONE      0x0Cu  
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_MUX_CONTROL_USB31     0x04u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_MUX_CONTROL_DP        0x08u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_MUX_CONTROL_USB31_D   0x00u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_CONNECTION_PRESENT    0x02u
#define TCPC_REG_CONFIG_STANDARD_OUTPUT_CONNECTOR_ORIENTATION 0x01u
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_TCPC_CONTROL TCPC REG_TCPC_CONTROL defines
  * @{
  */
#define TCPC_REG_TCPC_CONTROL_SET(__POLARITY__) (__POLARITY__)
#define TCPC_REG_TCPC_CONTROL_POLARITY(__REG__) ((__REG__) & 0x1)
#define TCPC_REG_TCPC_CONTROL_DEBUG_ACCESORY             0x10u
#define TCPC_REG_TCPC_CONTROL_DEBUG_I2C_CLOCKSTRETCHING  0x0Cu
#define TCPC_REG_TCPC_CONTROL_DEBUG_I2C_BISTMODE         0x02u
#define TCPC_REG_TCPC_CONTROL_DEBUG_PLUG_ORIENTATION     0x01u
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_ROLE_CONTROL TCPC REG_ROLE_CONTROL defines
  * @{
  */
#define TCPC_REG_ROLE_CONTROL_SET(__DRP__, __RP__, __CC1__, __CC2__) \
      ((__DRP__) << 6 | (__RP__) << 4 | (__CC2__) << 2 | (__CC1__))
#define TCPC_REG_ROLE_CONTROL_CC2(__REG__) (((__REG__) & 0xc) >> 2)
#define TCPC_REG_ROLE_CONTROL_CC1(__REG__) ((__REG__) & 0x3)
#define TCPC_REG_ROLE_CONTROL_MASK          0x7F
#define TCPC_REG_ROLE_CONTROL_DRP           0x40
#define TCPC_REG_ROLE_CONTROL_RP_MASK       (0x03 << 4)
#define TCPC_REG_ROLE_CONTROL_RP_DEFAULT    0x00
#define TCPC_REG_ROLE_CONTROL_RP_1_5A       (0x01 << 4)
#define TCPC_REG_ROLE_CONTROL_RP_3_0A       (0x02 << 4)
#define TCPC_REG_ROLE_CONTROL_CC2_MASK      (0x03 << 2)
#define TCPC_REG_ROLE_CONTROL_CC2_RA        (0x00 << 2)
#define TCPC_REG_ROLE_CONTROL_CC2_RP        (0x01 << 2)
#define TCPC_REG_ROLE_CONTROL_CC2_RD        (0x02 << 2)
#define TCPC_REG_ROLE_CONTROL_CC2_OPEN      (0x03 << 2)
#define TCPC_REG_ROLE_CONTROL_CC1_MASK      (0x03 << 0)
#define TCPC_REG_ROLE_CONTROL_CC1_RA        (0x00 << 0)
#define TCPC_REG_ROLE_CONTROL_CC1_RP        (0x01 << 0)
#define TCPC_REG_ROLE_CONTROL_CC1_RD        (0x02 << 0)
#define TCPC_REG_ROLE_CONTROL_CC1_OPEN      (0x03 << 0)
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_POWER_CONTROL TCPC POWER_CONTROL defines
  * @{
  */
#define TCPC_REG_POWER_CONTROL_SET(__VCONN__) (__VCONN__)
#define TCPC_REG_POWER_CONTROL_VCONN(__REG__)    ((__REG__) & 0x1)
#define TCPC_REG_POWER_CONTROL_ENABLE_VCONN               (0x01 << 0)
#define TCPC_REG_POWER_CONTROL_VCONN_POWER_SUPPORTED      (0x01 << 1)
#define TCPC_REG_POWER_CONTROL_FORCE_DISCHARGE            (0x01 << 2)
#define TCPC_REG_POWER_CONTROL_ENABLE_BLEED_DISCHARGE     (0x01 << 3)
#define TCPC_REG_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT  (0x01 << 4)
#define TCPC_REG_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS     (0x01 << 5)
#define TCPC_REG_POWER_CONTROL_VBUS_VOLTAGE_MONITOR       (0x01 << 6)
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_CC_STATUS TCPC REG_CC_STATUS defines
  * @{
  */
#define TCPC_REG_CC_STATUS_SET(__TERM__, __CC1__, __CC2__) \
      ((__TERM__) << 4 | ((__CC2__) & 0x3) << 2 | ((__CC1__) & 0x3))
#define TCPC_REG_CC_STATUS_TERM(__REG__) (((__REG__) & 0x10) >> 4)
#define TCPC_REG_CC_STATUS_CC2(__REG__)  (((__REG__) & 0xc) >> 2)
#define TCPC_REG_CC_STATUS_CC1(__REG__)  ((__REG__) & 0x3)
  
#define TCPC_REG_CC_STATUS_LOOKING4CONNECTION   (1u << 5)      
#define TCPC_REG_CC_STATUS_CONNECT_RP           0
#define TCPC_REG_CC_STATUS_CONNECT_RD           (1u << 4)
        
#define TCPC_REG_CC_STATUS_CC2_SRC_OPEN_RP      0
#define TCPC_REG_CC_STATUS_CC2_SRC_RA           (1u << 2)
#define TCPC_REG_CC_STATUS_CC2_SRC_RD           (2u << 2)
  
#define TCPC_REG_CC_STATUS_CC2_SNK_OPEN_RP      0
#define TCPC_REG_CC_STATUS_CC2_SNK_DEFAULT      (1u << 2)
#define TCPC_REG_CC_STATUS_CC2_SNK_1_5A         (2u << 2)
#define TCPC_REG_CC_STATUS_CC2_SNK_3_0A         (3u << 2)
  
#define TCPC_REG_CC_STATUS_CC1_SRC_OPEN_RP      0
#define TCPC_REG_CC_STATUS_CC1_SRC_RA           1u
#define TCPC_REG_CC_STATUS_CC1_SRC_RD           2u
  
#define TCPC_REG_CC_STATUS_CC1_SNK_OPEN_RP      0
#define TCPC_REG_CC_STATUS_CC1_SNK_DEFAULT      1u
#define TCPC_REG_CC_STATUS_CC1_SNK_1_5A         2u
#define TCPC_REG_CC_STATUS_CC1_SNK_3_0A         3u
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_POWER_STATUS TCPC REG_POWER_STATUS defines
  * @{
  */
#define TCPC_REG_POWER_STATUS_SINKING_VBUS              (1u << 0u)
#define TCPC_REG_POWER_STATUS_VCONN_PRESENT             (1u << 1u)
#define TCPC_REG_POWER_STATUS_VBUS_PRESENT              (1u << 2u)
#define TCPC_REG_POWER_STATUS_VBUS_PRESENT_ENABLED      (1u << 3u)
#define TCPC_REG_POWER_STATUS_SOURCING_VBUS             (1u << 4u)
#define TCPC_REG_POWER_STATUS_SOURCING_HIGH_VOLTAGE     (1u << 5u)
#define TCPC_REG_POWER_STATUS_TCPC_INIT_STATUS          (1u << 6u)
#define TCPC_REG_POWER_STATUS_DEBUG_ACCESSORY_CONNECTED (1u << 7u)
/**
  * @}
  */

/** @defgroup TCPC_EC_REG_FAULT_STATUS TCPC REG_FAULT_STATUS defines
  * @{
  */
#define TCPC_REG_FAULT_STATUS_VBUS_OFF         0b01000000u
#define TCPC_REG_FAULT_STATUS_AUTO_DISCHARGE   0b00100000u
#define TCPC_REG_FAULT_STATUS_FORCE_DISCHARGE  0b00010000u
#define TCPC_REG_FAULT_STATUS_OCP              0b00001000u
#define TCPC_REG_FAULT_STATUS_OVP              0b00000100u  
#define TCPC_REG_FAULT_STATUS_OVP              0b00000100u  
#define TCPC_REG_FAULT_STATUS_VCONN_OC         0b00000010u 
#define TCPC_REG_FAULT_STATUS_I2C              0x1u
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_COMMAND TCPC REG_COMMAND defines
  * @{
  */
#define  TCPC_REG_COMMAND_I2C_WAKE            0x11u
#define  TCPC_REG_COMMAND_DISABLE_VBUS_DETECT 0x22u
#define  TCPC_REG_COMMAND_ENABLE_VBUS_DETECT  0x33u
#define  TCPC_REG_COMMAND_DISABLE_SINK_VBUS   0x44u
#define  TCPC_REG_COMMAND_ENABLE_SINK_VBUS    0x55u
    
#define  TCPC_REG_COMMAND_DISABLE_SRC_VBUS    0x66u
#define  TCPC_REG_COMMAND_SRC_VBUS_DEFAULT    0x77u
#define  TCPC_REG_COMMAND_SRC_VBUS_HIGH       0x88u
    
#define  TCPC_REG_COMMAND_LOOK4CONNECTION     0x99u
#define  TCPC_REG_COMMAND_RXONEMORE           0xAAu
    
#define  TCPC_REG_COMMAND_I2C_IDLE            0xFFu
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_MSG_HEADER_INFO TCPC REG_MSG_HEADER_INFO defines
  * @{
  */
#define TCPC_REG_MSG_HEADER_INFO_SET(__DUALROLE__, __PROLE__) \
      ((__DUALROLE__) << 3 | (USBPD_SPECIFICATION_REV2 << 1) | (__PROLE__))
#define TCPC_REG_MSG_HEADER_INFO_DROLE(__REG__) (((__REG__) & 0x8) >> 3)
#define TCPC_REG_MSG_HEADER_INFO_PROLE(__REG__) ((__REG__) & 0x1)
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_RX_DETECT TCPC REG_RX_DETECT defines
  * @{
  */
#define TCPC_REG_RX_DETECT_SOP_HRST_MASK 0x21u
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_TRANSMIT TCPC REG_TRANSMIT defines
  * @{
  */
#define TCPC_REG_TRANSMIT_SET(__TYPE__, __RETRY__) \
      ((__RETRY__) << 4 | (__TYPE__))
#define TCPC_REG_TRANSMIT_RETRYCOUNTER(__VAL__)  ((__VAL__)&0x30u >> 4)
#define TCPC_REG_TRANSMIT_SOPMASK(__VAL__)       ((__VAL__)&0x07u)
#define TCPC_REG_TRANSMIT_SOP                0x00u
#define TCPC_REG_TRANSMIT_SOP1               0x01u
#define TCPC_REG_TRANSMIT_SOP2               0x02u
#define TCPC_REG_TRANSMIT_SOP_DBG1           0x03u
#define TCPC_REG_TRANSMIT_SOP_DBG2           0x04u
#define TCPC_REG_TRANSMIT_HARDRESET          0x05u
#define TCPC_REG_TRANSMIT_SOFTRESET          0x06u
#define TCPC_REG_TRANSMIT_BIST_CARRIER2      0x07u
/**
  * @}
  */
  
/** @defgroup TCPC_EC_REG_ALERT_VD TCPC REG_ALERT_VD defines
  * @{
  */
#define TCPC_REG_ALERT_VD_CLEAR_ALL          0x1Fu
#define TCPC_REG_ALERT_VD_I_DISCH_SUCC       (1 << 6) /*<! Auto discharge or force discharge successful   */
#define TCPC_REG_ALERT_VD_I_GPI2             (1 << 5) /*<! Input GPI2 change occurred                     */
#define TCPC_REG_ALERT_VD_I_GPI1             (1 << 4) /*<! Input GPI1 change occurred                     */
#define TCPC_REG_ALERT_VD_I_VDD_DTCT         (1 << 3) /*<! VDD detection change occurred (read VD_STAT)   */
#define TCPC_REG_ALERT_VD_I_OTP              (1 << 2) /*<! OTP condition occurred                         */
#define TCPC_REG_ALERT_VD_I_SWAP_TX          (1 << 1) /*<! Fast role swap sent due to GPIO input set low  */
#define TCPC_REG_ALERT_VD_I_SWAP_RX          (1 << 0) /*<! Fast role swap request received                */
/**
  * @}
  */

typedef enum {
  TCPC_REG_VENDOR_ID                   = 0x00u,
  TCPC_REG_PRODUCT_ID                  = 0x02u,
  TCPC_REG_DEVICE_ID                   = 0x04u,
  TCPC_REG_USBTYPEC_REV                = 0x06u,
  TCPC_REG_USBPD_REV_VER               = 0x08u,
  TCPC_REG_USBPD_INTERFACE_REV         = 0x0Au,
  
  TCPC_REG_ALERT                       = 0x10u, /*<! Can be set to @ref TCPC_EC_REG_ALERT                   */
  TCPC_REG_ALERT_MASK                  = 0x12u, /*<! Can be set to @ref TCPC_EC_REG_ALERT_MASK              */

  TCPC_REG_POWER_STATUS_MASK           = 0x14u, /*<! Can be set to @ref TCPC_EC_REG_POWER_STATUS_MASK       */
  TCPC_REG_FAULT_STATUS_MASK           = 0x15u, /*<! Can be set to @ref TCPC_EC_REG_FAULT_STATUS_MASK       */
  
  TCPC_REG_CONFIG_STANDARD_OUTPUT      = 0x18u, /*<! Can be set to @ref TCPC_EC_REG_CONFIG_STANDARD_OUTPUT  */

  
  TCPC_REG_TCPC_CONTROL                = 0x19u, /*<! Can be set to @ref TCPC_EC_REG_TCPC_CONTROL            */
  
  TCPC_REG_ROLE_CONTROL                = 0x1Au, /*<! Can be set to @ref TCPC_EC_REG_ROLE_CONTROL            */
  
  TCPC_REG_FAULT_CONTROL               = 0x1Bu,
  TCPC_REG_POWER_CONTROL               = 0x1Cu, /*<! Can be set to @ref TCPC_EC_REG_POWER_CONTROL           */

  TCPC_REG_CC_STATUS                   = 0x1Du, /*<! Can be set to @ref TCPC_EC_REG_CC_STATUS               */
      
  TCPC_REG_POWER_STATUS                = 0x1Eu, /*<! Can be set to @ref TCPC_EC_REG_POWER_STATUS            */

  TCPC_REG_FAULT_STATUS                = 0x1Fu, /*<! Can be set to @ref TCPC_EC_REG_FAULT_STATUS            */
  
  TCPC_REG_COMMAND                     = 0x23u, /*<! Can be set to @ref TCPC_EC_REG_COMMAND                 */
  
  TCPC_REG_DECIVE_CAP1                 = 0x24u,
  TCPC_REG_DECIVE_CAP2                 = 0x26u,
  TCPC_REG_STD_INPUT_CAP               = 0x28u,
  TCPC_REG_STD_OUTPUT_CAP              = 0x29u,
  
  TCPC_REG_MSG_HEADER_INFO             = 0x2Eu, /*<! Can be set to @ref TCPC_EC_REG_MSG_HEADER_INFO         */

  TCPC_REG_RX_DETECT                   = 0x2Fu, /*<! Can be set to @ref TCPC_EC_REG_RX_DETECT               */
  TCPC_REG_RX_BYTE_COUNT               = 0x30u,
  TCPC_REG_RX_BUFFER_FRAME_TYPE        = 0x31u,
  TCPC_REG_RX_HEADER                   = 0x32u,
  TCPC_REG_RX_DATA                     = 0x34u,
  
  TCPC_REG_TRANSMIT                    = 0x50u, /*<! Can be set to @ref TCPC_EC_REG_TRANSMIT                */
  
  TCPC_REG_TX_BYTE_COUNT               = 0x51u,
  TCPC_REG_TX_HEADER                   = 0x52u,
  TCPC_REG_TX_DATA                     = 0x54u,

  TCPC_REG_VBUS_VOLTAGE                   = 0x70u,
  TCPC_REG_VBUS_SINK_DISCONNECT_THRESHOLD = 0x72u,
  TCPC_REG_VBUS_STOP_DISCHARGE_THRESHOLD  = 0x74u,
  TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG      = 0x76u,
  TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG      = 0x78u,

  TCPC_REG_VENDOR_DATA                    = 0x80u,
} USBPD_TCPC_RegisterId;

/**
  * @}
  */

/** @defgroup TCPC_Exported_Types TCPC Exported Types
  * @{
  */

typedef union {
 uint16_t word[6];
 struct {
   uint16_t VENDOR_ID;
   uint16_t PRODUCT_ID;
   uint16_t DEVICE_ID;
   uint16_t USBTYPEC_REV;
   uint16_t USBPD_REV_VER;
   uint16_t PD_INTERFACE_REV;
 }b;
} regInformation_t;

typedef union {
  uint16_t word[2];
  uint8_t  byte[4];
  struct {
    /* ALERTL LOW */
    union {
      uint8_t ALERTL;
      struct {
        uint8_t I_CCSTAT:1;
        uint8_t I_PORT_PWR:1;
        uint8_t I_RXSTAT:1;
        uint8_t I_RXHRDRST:1;
        uint8_t I_TXFAIL:1;
        uint8_t I_TXDISC:1;
        uint8_t I_TXSUCC:1;
        uint8_t I_VBUS_ALRM_HI:1;
      }b1;
    }u1;
    /* ALERT HIGH */
    union {
      uint8_t ALERTH;
      struct {
        uint8_t I_VBUS_ALRM_LO:1;
        uint8_t I_FAULT:1;
        uint8_t I_RX_FULL:1;
        uint8_t I_VBUS_SNK_DISC:1;
        uint8_t I_Reserved:3;
        uint8_t I_VD_ALERT:1;
      }b2;
    }u2;
    /* ALERT MASK LOW */
    union {
      uint8_t ALERTMSKL;
      struct {
        uint8_t M_CCSTAT:1;
        uint8_t M_PORT_PWR:1;
        uint8_t M_RXSTAT:1;
        uint8_t M_RXHRDRST:1;
        uint8_t M_TXFAIL:1;
        uint8_t M_TXDISC:1;
        uint8_t M_TXSUCC:1;
        uint8_t M_VBUS_ALRM_HI:1;
      }b3;
    }u3;
    /* ALERT MASK HIGH */
    union {
      uint8_t ALERTMSKH;
      struct {
        uint8_t M_VBUS_ALRM_LO:1;
        uint8_t M_FAULT:1;
        uint8_t M_RX_FULL:1;
        uint8_t M_VBUS_SNK_DISC:1;
        uint8_t M_Reserved:3;
        uint8_t M_VD_ALERT:1;
      }b4;
    }u4;
  }s;
} regAlert_t;

typedef union {
  uint8_t byte[2];
  struct {
    /* POWER_STATUS_MASK */
    union {
      uint8_t POWER_STATUS_MASK;
      struct {
        uint8_t M_SNKVBUS:1;
        uint8_t M_VCONN_VAL:1;
        uint8_t M_VBUS_VAL:1;
        uint8_t M_VBUS_VAL_EN:1;
        uint8_t M_SRC_VBUS:1;
        uint8_t M_SRC_HV:1;
        uint8_t M_INIT:1;
        uint8_t M_DEBUG_ACC:1;
      }b1;
    }u1;
    /* FAULT_STATUS_MASK */
    union {
      uint8_t FAULT_STATUS_MASK;
      struct {
        uint8_t M_I2C_ERR:1;
        uint8_t M_VCONN_OCP:1;
        uint8_t M_Reserved1:2;
        uint8_t M_FORCE_DISCH_FAIL:1;
        uint8_t M_AUTO_DISCH_FAIL:1;
        uint8_t M_Reserved2:1;
        uint8_t M_ALL_REGS_RESET:1;
      }b2;
    }u2;
  }s;
} regStatusMask_t;

typedef union {
  uint8_t byte[5];
  struct {
    union {
      uint8_t CONFIG_STANDARD_OUTPUT;
      struct {
        uint8_t CONNECTOR_ORIENT:1;
        uint8_t CSO_Reserved1:1;
        uint8_t MUX_CTRL:2;
        uint8_t CSO_Reserved2:2;
        uint8_t DEBUG_ACC:1;
        uint8_t TRI_STATE:1;
      }b1;
    }u1;
    union {
      uint8_t TCPC_CONTROL;
      struct {
        uint8_t PLUG_ORIENT:1;
        uint8_t BIST_TMODE:1;
        uint8_t I2C_CLK_STRETCH:2;
        uint8_t DEBUG_ACC_CTRL:1;
        uint8_t EN_WATCHDOG:1;
        uint8_t TC_Reserved:2;
      }b2;
    }u2;
    union {
      uint8_t ROLE_CONTROL;
      struct {
        uint8_t CC1_TERM:2;
        uint8_t CC2_TERM:2;
        uint8_t RP_VAL:2;
        uint8_t DRP:1;
        uint8_t RC_Reserved:1;
      }b3;
    }u3;
    union {
      uint8_t FAULT_CONTROL;
      struct {
        uint8_t VCONN_OCP_DIS:1;
        uint8_t FC_Reserved1:2;
        uint8_t DISCH_TIMER_DIS:1;
        uint8_t FC_Reserved2:4;
      }b4;
    }u4;
    union {
      uint8_t POWER_CONTROL;
      struct {
        uint8_t EN_VCONN:1;
        uint8_t VCONN_PWR:1;
        uint8_t FORCE_DISCH:1;
        uint8_t EN_BLEED_DISCH:1;
        uint8_t AUTO_DISCH:1;
        uint8_t DIS_VALRM:1;
        uint8_t VBUS_MON:1;
        uint8_t PC_Reserved:1;
      }b5;
    }u5;
  }s;
} regControl_t;

typedef struct {
    /* CC_STATUS */
    union {
      uint8_t CC_STATUS;
      struct {
        uint8_t CC1_STAT:2;
        uint8_t CC2_STAT:2;
        uint8_t CON_RES:1;
        uint8_t LOOK4CON:1;
        uint8_t CC_Reserved:2;
      }b1;
    }u1;
    /* POWER_STATUS */
    union {
      uint8_t POWER_STATUS;
      struct {
        uint8_t SNKVBUS:1;
        uint8_t VCONN_VAL:1;
        uint8_t VBUS_VAL:1;
        uint8_t VBUS_VAL_EN:1;
        uint8_t SOURCE_VBUS:1;
        uint8_t SOURCE_HV:1;
        uint8_t TCPC_INIT:1;
        uint8_t DEBUG_ACC:1;
      }b2;
    }u2;
    /* FAULT_STATUS */
    union {
      uint8_t FAULT_STATUS;
      struct {
        uint8_t I2C_ERR:1;
        uint8_t VCONN_OCP:1;
        uint8_t Reserved1:2;
        uint8_t FORCE_DISCH_FAIL:1;
        uint8_t AUTO_DISCH_FAIL:1;
        uint8_t Reserved2:1;
        uint8_t ALL_REGS_RESET:1;
      }b3;
    }u3;
} regStatus_t;

typedef union {
  uint8_t byte;
  struct {
    uint8_t COMMAND;
  }u;
} regCommand_t;

typedef union {
  uint16_t word[3];
  uint8_t byte[6];
  struct {
    union {
      uint16_t DEVICE_CAPABILITIES_1;
      struct {
        uint16_t SourceVBUS:1;
        uint16_t SourceHighVoltageVBUS:1;
        uint16_t SinkVBUS:1;
        uint16_t SourceVCONN:1;
        uint16_t SOP_Support:1;
        uint16_t RoleSupported:3;
        uint16_t SourceResistorSupported:2;
        uint16_t VBUS_MeasAndAlarm:1;
        uint16_t ForceDischarge:1;
        uint16_t BleedDischarge:1;
        uint16_t VBUS_OVP_Reporting:1;
        uint16_t VBUS_OCP_Reporting:1;
        uint16_t DC1_Reserved:1;
      }b1;
    }u1;
    union {
      uint16_t DEVICE_CAPABILITIES_2;
      struct {
        uint16_t VCONN_OverCurrFaultCapable:1;
        uint16_t VCONN_PowerSupported:3;
        uint16_t VBUS_VoltAlarmLSB:2;
        uint16_t StopDischargeThreshold:1;
        uint16_t SinkDisconnectDetection:1;
        uint16_t WatchdogTimer:1;
        uint16_t DC2_Reserved:7;
      }b2;
    }u2;
    union {
      uint8_t  STANDARD_INPUT_CAPABILITIES;
      struct {
        uint8_t ForceOffVBUS:1;
        uint8_t VBUS_ExtOverCurrentFault:1;
        uint8_t VBUS_ExtOverVoltageFault:1;
        uint8_t SIC_Reserved:5;
      }b3;
    }u3;
    union {
      uint8_t  STANDARD_OUTPUT_CAPABILITIES;
      struct {
        uint8_t ConnectorOrentation:1;
        uint8_t ConnectionPresent:1;
        uint8_t MUX_ConfigControl:1;
        uint8_t ActiveCableIndicator:1;
        uint8_t AudioAdapterAccessoryIndicator:1;
        uint8_t VBUS_PresentMonitor:1;
        uint8_t DebugAccessoryIndicator:1;
        uint8_t SOC_Reserved:1;
      }b4;
    }u4;
  }s;
} regCapability_t;

typedef struct {
    union {
      uint8_t MESSAGE_HEADER_INFO;
      struct {
        uint8_t PWR_ROLE:1;
        uint8_t USB_PD_REV:2;
        uint8_t DATA_ROLE:1;
        uint8_t CABLE_PLUG:1;
        uint8_t MH_Reserved:3;
      }b1;
    }u1;
    union {
      uint8_t RECEIVE_DETECT;
      struct {
        uint8_t EN_SOP:1;
        uint8_t EN_SOP1:1;
        uint8_t EN_SOP2:1;
        uint8_t EN_SOP1_DBG:1;
        uint8_t EN_SOP2_DBG:1;
        uint8_t EN_HRD_RST:1;
        uint8_t EN_CABLE_RST:1;
        uint8_t RD_Reserved:1;
      }b2;
    }u2;
} regFrameInfo_t;

typedef union {
  uint8_t byte[4];
  struct {
    uint8_t  RECEIVE_BYTE_COUNT;
    uint8_t  RX_BUF_FRAME_TYPE;
    uint8_t  RX_BUF_HEADER_BYTE_0;
    uint8_t  RX_BUF_HEADER_BYTE_1;
//    uint8_t  RX_BUF_OBJ[28];
  }b;
} regRXFrame_t;

typedef union {
  uint8_t byte[4];
  struct {
    uint8_t  TRANSMIT;
    uint8_t  TRANSMIT_BYTE_COUNT;
    uint8_t  TX_BUF_HEADER_BYTE_0;
    uint8_t  TX_BUF_HEADER_BYTE_1;
//    uint8_t  TX_BUF_OBJ[28];
  }u;
} regTXFrame_t;

typedef union {
  uint16_t word[5];
  struct {
    union {
      uint16_t VBUS_VOLTAGE;
      struct {
        uint16_t VBUS_Measurement :10;  /* !< VBUS measurement */
        uint16_t VBUS_Scale       :2;   /* !< VBUS measurement scale (0: not scaled, 1: div by 2, 2: div by 4) */
        uint16_t VBUS_Reserved    :4;   /* !< Reserved bits */
      }b;
    }u;
    uint16_t  VBUS_SINK_DISCONNECT_THRESHOLD;
    uint16_t  VBUS_STOP_DISCHARGE_THRESHOLD;
    uint16_t  VBUS_VOLTAGE_ALARM_HI_CFG;
    uint16_t  VBUS_VOLTAGE_ALARM_LO_CFG;
  }s;
} regVBUS_t;

typedef struct 
{
  regInformation_t    TCPC_Information;
  regAlert_t          Alerts;
  regStatusMask_t     StatusMask;
  regControl_t        Control;
  regStatus_t         Status;
  regCommand_t        Command;
  regCapability_t     Capabilities;
  regFrameInfo_t      FrameInfo;
  regRXFrame_t        RXFrame;
  regTXFrame_t        TXFrame;
  regVBUS_t           VBUS;
} DeviceReg_t;

/** @defgroup TCPC_Driver_structure  TCPC Driver structure
  * @{
  */
typedef struct
{
  /**
    * @brief  Initialize TCPC device
    * @param  PortNum       PortNum number value
    * @param  Role          Power role based on @ref USBPD_PortPowerRole_TypeDef
    * @param  ToggleRole    Indicate TCPC to enable automatically the Toggle role
    * @param  IsSwapOngoing Callback function to check if PRS is ongoing on PE side.
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*init)(uint32_t PortNum, USBPD_PortPowerRole_TypeDef Role, uint8_t ToggleRole, uint8_t (*IsSwapOngoing)(uint8_t));
  /**
    * @brief  Get CC line for PD connection
    * @param  PortNum       PortNum number value
    * @param  CC1_Level     Pointer of status of the CC1 line
    * @param  CC2_Level     Pointer of status of the CC2 line
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*get_cc)(uint32_t PortNum, uint32_t *CC1_Level, uint32_t *CC2_Level);
  /**
    * @brief  Get Power status level
    * @param  PortNum       PortNum number value
    * @param  PowerStatus Pointer on Power status
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*get_power_status)(uint32_t PortNum, uint8_t *PowerStatus);
  /**
    * @brief  Get fault status register
    * @param  PortNum       PortNum number value
    * @param  FaultStatus   Pointer on Fault status register
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*get_fault_status)(uint32_t PortNum, uint8_t *FaultStatus);
  /**
    * @brief  Set fault status register
    * @param  PortNum       PortNum number value
    * @param  FaultStatus   Fault status register
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_fault_status)(uint32_t PortNum, uint8_t FaultStatus);
  /**
    * @brief  Get VBUS level
    * @param  PortNum       PortNum number value
    * @param  VBUSLevel     Pointer on VBUS level
    * @param  VBUSVoltage   Pointer on VBUS voltage
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*get_vbus_level)(uint32_t PortNum, uint8_t *VBUSLevel, uint16_t *VBUSVoltage);
  /**
    * @brief  Set VBUS level
    * @param  PortNum       PortNum number value
    * @param  Enable        Enable or disable VBUS level based on @ref USBPD_FunctionalState
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_vbus_level)(uint32_t PortNum, USBPD_FunctionalState State);
  /**
    * @brief  Set CC line for PD connection
    * @param  PortNum       PortNum number value
    * @param  Pull          Power role based on @ref TCPC_CC_Pull_TypeDef
    * @param  State         State of the connection based on @ref USBPD_FunctionalState
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_cc)(uint32_t PortNum, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State);
  /**
    * @brief  Set the polarity of the CC lines
    * @param  PortNum       PortNum number value
    * @param  Polarity      Polarity
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_polarity)(uint32_t PortNum, uint8_t Polarity);
  /**
    * @brief  Enable or disable VCONN
    * @param  PortNum       PortNum number value
    * @param  State         Activation or deactivation of VCONN (@ref USBPD_FunctionalState)
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_vconn)(uint32_t PortNum, USBPD_FunctionalState State);
  /**
    * @brief  Set power and data role et PD message header
    * @param  PortNum       PortNum number value
    * @param  PowerRole     Power role based on @ref USBPD_PortPowerRole_TypeDef
    * @param  DataRole      Data role based on @ref USBPD_PortDataRole_TypeDef
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_msg_header)(uint32_t PortNum, USBPD_PortPowerRole_TypeDef PowerRole, USBPD_PortDataRole_TypeDef DataRole);
  /**
    * @brief  Enable or disable PD reception
    * @param  PortNum       PortNum number value
    * @param  Enable        Activation or deactivation of RX
    * @param  SupportedSOP  Supported SOP by PRL
    * @param  HardReset     Hard reset status based on @ref TCPC_hard_reset
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_rx_state)(uint32_t PortNum, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State, uint32_t SupportedSOP, TCPC_hard_reset HardReset);
  /**
    * @brief  Set SOP supported
    * @param  PortNum       PortNum number value
    * @param  SupportedSOP  Supported SOP by PRL
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*set_sop_supported)(uint32_t PortNum, uint32_t SupportedSOP);
  /**
    * @brief  Retrieve the PD message 
    * @param  PortNum       PortNum number value
    * @param  Payload Pointer on the payload
    * @param  Type Pointer on the message type
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*get_message)(uint32_t PortNum, uint8_t *payload, uint8_t *Type);
  /**
    * @brief  Management of ALERT
    * @param  PortNum       PortNum number value
    * @param  Alert Pointer on ALERT
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*tcpc_alert)(uint32_t PortNum, uint16_t *Alert);
  /**
    * @brief  Clear ALERT on TCPC device
    * @param  PortNum       PortNum number value
    * @param  Alert Pointer on ALERT to clear
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*tcpc_clear_alert)(uint32_t PortNum, uint16_t *Alert);
  /**
    * @brief  Transmit the PD message 
    * @param  PortNum       PortNum number value
    * @param  Type Message type
    * @param  Header Header of the PD message
    * @param  pData Pointer on the data message
    * @param  RetryNumber Number of retry
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*transmit)(uint32_t PortNum, USBPD_SOPType_TypeDef Type, uint16_t header, const uint8_t *data, uint32_t RetryNumber);
  /**
    * @brief  Set BIST data managment
    * @param  PortNum       PortNum number value
    * @param  Enable Enable BIST Carrier mode 2 or not
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*tcpc_send_bist_pattern)(uint32_t PortNum, uint8_t Enable);

  /**
   * @brief  function to set the SinkTxNg
   * @param  PortNum       PortNum number value
   * @retval USBPD status
    */
  USBPD_StatusTypeDef (*tcpc_SinkTxNG)(uint32_t PortNum);
  /**
   * @brief  function to set the SinkTxOK
   * @param  PortNum       PortNum number value
   * @retval USBPD status
    */
  USBPD_StatusTypeDef (*tcpc_SinkTxOK)(uint32_t PortNum);

  /**
   * @brief  function to check if SinkTxOK
   * @param  PortNum  Number of the port.
   * @retval USBPD status based on @ref USBPD_StatusTypeDef
    */
  USBPD_StatusTypeDef (*tcpc_IfSinkTxOK)(uint32_t PortNum);
}TCPC_DrvTypeDef;

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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __TCPC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
