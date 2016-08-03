/**
  ******************************************************************************
  * @file    usbpd_def.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   Global defines for USB-PD libarary
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
#ifndef __USBPD_DEF_H_
#define __USBPD_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common_defines.h"

/** @addtogroup USB-PD 
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/**
  * @brief USB Power Delivery Status structures definition
  */
typedef enum
{
  USBPD_OK         = 0x00,
  USBPD_ERROR      = 0x01,
  USBPD_BUSY       = 0x02,
  USBPD_TIMEOUT    = 0x03,
  USBPD_ACK        = 0x04,
  USBPD_NAK        = 0x05,
  USBPD_GOODCRC    = 0x06,
  USBPD_FAIL       = 0x07
}USBPD_StatusTypeDef;

/**
  * @brief USB PD Rx Phy Status structures definition
  */
typedef enum 
{
  RX_OK,
  RX_ERROR,
  RX_ERROR_WRONG_ORDERSET,
  RX_ERROR_WRONG_SYMBOL,
  RX_ERROR_MISSING_EOP,
  RX_ERROR_CRC_FAILED
}USBPD_PHY_RX_StatusTypeDef;

/**
  * @brief USB PD CC lines structures definition
  */
typedef enum
{
  CCNONE      = 0,
  CC1         = 1,
  CC2         = 2
}CCxPin_TypeDef;


#define OTHER_CC(__CC__)                ( ((CCxPin_TypeDef)(__CC__))==CC1 ? CC2 : ( (((CCxPin_TypeDef)(__CC__))==CC2) ? CC1 : CCNONE ) ) 


/** @defgroup USBPD_SpecRev_TypeDef USB PD Specification Revision structure definition
  * @brief  USB PD Specification Revision structure definition
  * @{
  */
typedef enum
{
  USBPD_SPECIFICATION_REV1               = 0x00,  /*!< Revision 1.0      */
  USBPD_SPECIFICATION_REV2               = 0x01   /*!< Revision 2.0      */
}USBPD_SpecRev_TypeDef;
/** 
  * @}
  */


/**
  * @brief USB PD CC lines HW condition
  */
typedef enum
{
  HW_Detachment                         = 0x00,    /*!< Nothing attached   */
  HW_Attachment                         = 0x01,    /*!< Sink attached   */    
  HW_PwrCable_NoSink_Attachment         = 0x02,    /*!< Powered cable without Sink attached   */
  HW_PwrCable_Sink_Attachment           = 0x03,    /*!< Powered cable with Sink or VCONN-powered Accessory attached   */
  HW_Debug_Attachment                   = 0x04,    /*!< Debug Accessory Mode attached   */
  HW_AudioAdapter_Attachment            = 0x05     /*!< Audio Adapter Accessory Mode attached   */ 
}CAD_HW_Condition_TypeDef;


/** @defgroup USBPD_PortDataRole_TypeDef USB PD Port Data Role Types structure definition
  * @brief  USB PD Port Data Role Types structure definition
  * @{
  */
typedef enum
{
  USBPD_PORTDATAROLE_UFP                 = 0x00,  /*!< UFP        */
  USBPD_PORTDATAROLE_DFP                 = 0x01   /*!< DFP        */
}USBPD_PortDataRole_TypeDef;
/** 
  * @}
  */

/**
  * @brief Sink CC pins Multiple Source Current Advertisements
  */
typedef enum
{
  vRd_Undefined     = 0x00,    /*!< Port Power Role Source   */
  vRd_USB           = 0x01,    /*!< Default USB Power   */    
  vRd_1_5A          = 0x02,    /*!< USB Type-C Current @ 1.5 A   */
  vRd_3_0A          = 0x03,    /*!< USB Type-C Current @ 3 A   */
}CAD_SNK_Source_Current_Adv_Typedef;


/**
  * @brief USB PD SOP Message Types Structure definition
  */
typedef enum
{
  USBPD_MSGTYPE_SOP            = 0,     /**< SOP*  MESSAGES               */
  USBPD_MSGTYPE_SOP1           = 1,     /**< SOP'  MESSAGES               */
  USBPD_MSGTYPE_SOP2           = 2,     /**< SOP'' MESSAGES               */
  USBPD_MSGTYPE_SOP1_DEBUG     = 3,     /**< SOP'  DEBUG_MESSAGES         */
  USBPD_MSGTYPE_SOP2_DEBUG     = 4,     /**< SOP'' DEBUG_MESSAGES         */
  USBPD_MSGTYPE_HARD_RESET     = 5,     /**< HARD RESET MESSAGE           */
  USBPD_MSGTYPE_CABLE_RESET    = 6,     /**< CABLE RESET MESSAGE          */
  USBPD_MSGTYPE_INVALID        = 0xFF,  /**< Invalid type                 */
} USBPD_MsgType_TypeDef;

/**
  * @brief USB PD Port Power Role Types structure definition
  *
  */
typedef enum
{
  USBPD_PORTPOWERROLE_SNK      = 0x00,  /*!< Sink                         */
  USBPD_PORTPOWERROLE_SRC      = 0x01,  /*!< Source                       */
  USBPD_PORTPOWERROLE_DRP_SRC  = 0x03,  /*!< DR Device starting as Source */
  USBPD_PORTPOWERROLE_DRP_SNK  = 0x04   /*!< DR Device starting as Sink   */
}USBPD_PortPowerRole_TypeDef;

/**
  * @brief  USB PD Data Message Types structure definition
  *
  */
typedef enum
{
  USBPD_DATAMSG_SRC_CAPABILITIES         = 0x01,  /*!< Source Capabilities Data Message  */
  USBPD_DATAMSG_REQUEST                  = 0x02,  /*!< Request Data Message              */
  USBPD_DATAMSG_BIST                     = 0x03,  /*!< BIST Data Message                 */
  USBPD_DATAMSG_SNK_CAPABILITIES         = 0x04,  /*!< Sink_Capabilities Data Message    */
  USBPD_DATAMSG_VENDOR_DEFINED           = 0x0F   /*!< Vendor_Defined Data Message       */
}USBPD_DataMsg_TypeDef;

/**
  * @brief  USB PD Control Message Types structure definition
  *
  */
typedef enum
{
  USBPD_CONTROLMSG_GOODCRC               = 0x01,  /*!< GoodCRC Control Message         */
  USBPD_CONTROLMSG_GOTOMIN               = 0x02,  /*!< GotoMin Control Message         */
  USBPD_CONTROLMSG_ACCEPT                = 0x03,  /*!< Accept Control Message          */
  USBPD_CONTROLMSG_REJECT                = 0x04,  /*!< Reject Control Message          */
  USBPD_CONTROLMSG_PING                  = 0x05,  /*!< Ping Control Message            */
  USBPD_CONTROLMSG_PS_RDY                = 0x06,  /*!< PS_RDY Control Message          */
  USBPD_CONTROLMSG_GET_SRC_CAP           = 0x07,  /*!< Get_Source_Cap Control Message  */
  USBPD_CONTROLMSG_GET_SNK_CAP           = 0x08,  /*!< Get_Sink_Cap Control Message    */
  USBPD_CONTROLMSG_DR_SWAP               = 0x09,  /*!< DR_Swap Control Message         */
  USBPD_CONTROLMSG_PR_SWAP               = 0x0A,  /*!< PR_Swap Control Message         */
  USBPD_CONTROLMSG_VCONN_SWAP            = 0x0B,  /*!< VCONN_Swap Control Message      */
  USBPD_CONTROLMSG_WAIT                  = 0x0C,  /*!< Wait Control Message            */
  USBPD_CONTROLMSG_SOFT_RESET            = 0x0D   /*!< Soft_Reset Control Message      */
}USBPD_ControlMsg_TypeDef;

/**
  * @brief  USB PD BIST Mode Types structure definition
  *
  */
typedef enum
{
  USBPD_BIST_RECEIVER_MODE               = 0x00,  /*!< BIST Receiver Mode      */
  USBPD_BIST_TRANSMIT_MODE               = 0x01,  /*!< BIST Transmit Mode      */
  USBPD_RETURNED_BIST_COUNTERS           = 0x02,  /*!< Returned BIST Counters  */
  USBPD_BIST_CARRIER_MODE0               = 0x03,  /*!< BIST Carrier Mode 0     */
  USBPD_BIST_CARRIER_MODE1               = 0x04,  /*!< BIST Carrier Mode 1     */
  USBPD_BIST_CARRIER_MODE2               = 0x05,  /*!< BIST Carrier Mode 2     */
  USBPD_BIST_CARRIER_MODE3               = 0x06,  /*!< BIST Carrier Mode 3     */
  USBPD_BIST_EYE_PATTERN                 = 0x07,  /*!< BIST Eye Pattern        */
  USBPD_BIST_TEST_DATA                   = 0x08   /*!< BIST Test Data          */
}USBPD_BISTMsg_TypeDef;

/**
  * @brief USB PD Message header Structure definition
  */
typedef union
{
  uint16_t d16;
  struct
  {
    uint16_t MessageType :			   /*!< Message Header's message Type                      */
      4;
    uint16_t Reserved4 :               /*!< Reserved                                           */
      1;
    uint16_t PortDataRole :			   /*!< Message Header's Port Data Role     			   */
      1;                                    
    uint16_t SpecificationRevision :   /*!< Message Header's Spec Revision     			       */
      2;       
    uint16_t PortPowerRole_CablePlug : /*!< Message Header's Port Power Role/Cable Plug field  */
      1;       
    uint16_t MessageID :               /*!< Message Header's message ID     				   */
      3;    
    uint16_t NumberOfDataObjects :     /*!< Message Header's Number of data object     		   */
      3;                
    uint16_t Reserved15 :              /*!< Reserved     						               */
      1; 
  }
  b;
}USBPD_MsgHeader_TypeDef;


/**
  * @brief  USB PD Source Fixed Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxCurrentIn10mAunits :
        10;
    uint32_t VoltageIn50mVunits :
        10;
    uint32_t PeakCurrent :
        2;
    uint32_t Reserved22_24 :
        3;
    uint32_t DataRoleSwap :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t ExternallyPowered :
        1;
    uint32_t USBSuspendSupported :
        1;
    uint32_t DualRolePower :
        1;
    uint32_t FixedSupply :
        2;
  }
  b;
}USBPD_SRCFixedSupplyPDO_TypeDef;


/**
  * @brief  USB PD Source Variable Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxCurrentIn10mAunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t VariableSupply :
        2;
  }
  b;
}USBPD_SRCVariableSupplyPDO_TypeDef;


/**
  * @brief  USB PD Source Battery Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxAllowablePowerIn250mWunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t Battery :
        2;
  }
  b;
}USBPD_SRCBatterySupplyPDO_TypeDef;


/**
  * @brief  USB PD Sink Fixed Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalCurrentIn10mAunits :
        10;
    uint32_t VoltageIn50mVunits :
        10;
    uint32_t Reserved20_24 :
        5;
    uint32_t DataRoleSwap :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t ExternallyPowered :
        1;
    uint32_t HigherCapability :
        1;
    uint32_t DualRolePower :
        1;
    uint32_t FixedSupply :
        2;
  }
  b;
}USBPD_SNKFixedSupplyPDO_TypeDef;


/**
  * @brief  USB PD Sink Variable Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalCurrentIn10mAunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t VariableSupply :
        2;
  }
  b;
}USBPD_SNKVariableSupplyPDO_TypeDef;


/**
  * @brief  USB PD Sink Battery Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalPowerIn250mWunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t Battery :
        2;
  }
  b;
}USBPD_SNKBatterySupplyPDO_TypeDef;


/**
  * @brief  USB PD Sink Fixed and Variable Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxOperatingCurrent10mAunits :
        10;
    uint32_t OperatingCurrentIn10mAunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKFixedVariableRDO_TypeDef;


/**
  * @brief  USB PD Sink Fixed and Variable Request Data Object with GiveBack Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MinOperatingCurrent10mAunits :
        10;
    uint32_t OperatingCurrentIn10mAunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKFixedVariableRDOGiveBack_TypeDef;


/**
  * @brief  USB PD Sink Battery Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxOperatingPowerIn250mWunits :
        10;
    uint32_t OperatingPowerIn250mWunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKBatteryRDO_TypeDef;


/**
  * @brief  USB PD Sink Battery with GiveBack Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MinOperatingPowerIn250mWunits :
        10;
    uint32_t OperatingPowerIn250mWunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKBatteryGiveBackRDO_TypeDef;


/**
  * @brief  USB PD BIST Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {  
    uint32_t BistErrorCounter :
      16;
    uint32_t Reserved16_27 :
      12;
    uint32_t BistMode : 
      4;
  }
  b;
}USBPD_BISTDataObject_TypeDef;


/* Exported constants --------------------------------------------------------*/
#define USBPD_MAX_NB_PDO       ((uint32_t)7)  /*!< Maximum number of supported Power Data Objects: fix by the Specification */
#define BIST_CARRIER_MODE_MS   ((uint32_t)50) /*!< Time in ms of the BIST signal*/

/*
  * Maximum size of a Power Delivery packet (in bits on the wire) :
  *    16-bit header + 0..7 32-bit data objects  (+ 4b5b encoding)
  *    64-bit preamble + SOP (4x 5b) + header (16-bit) + message in 4b5b + 32-bit CRC  + EOP (1x 5b)
  * =  64bit           + 4*5bit      + 16bit * 5/4 + 7 * 32bit * 5/4 + 32bit * 5/4 + 5
  */
#define PHY_BIT_LEN            ((uint16_t)429)
#define PHY_MAX_RAW_SIZE       ((uint16_t)((PHY_BIT_LEN*2) + 3))

/* Exported macro ------------------------------------------------------------*/
#define LE16(addr) (((uint16_t)(*((uint8_t *)(addr))))\
                    + (((uint16_t)(*(((uint8_t *)(addr)) + 1))) << 8))

#define LE32(addr) ((((uint32_t)(*(((uint8_t *)(addr)) + 0))) + \
                    (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 8) + \
                    (((uint32_t)(*(((uint8_t *)(addr)) + 2))) << 16) + \
                    (((uint32_t)(*(((uint8_t *)(addr)) + 3))) << 24)))

#define USBPD_PORTPOWERROLE_IS_SRC(val) (((val) == USBPD_PORTPOWERROLE_SRC) || ((val) == USBPD_PORTPOWERROLE_DRP_SRC))
#define USBPD_PORTPOWERROLE_IS_SNK(val) (((val) == USBPD_PORTPOWERROLE_SNK) || ((val) == USBPD_PORTPOWERROLE_DRP_SNK))

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DEF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
