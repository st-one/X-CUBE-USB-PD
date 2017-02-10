/**
  ******************************************************************************
  * @file    usbpd_vdm.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains the headers of usbpd_vdm.h.
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

#ifndef __USBPD_VDM_H_
#define __USBPD_VDM_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_prl.h"
#include "string.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_VDM
  * @{
  */

/* Exported Defines ----------------------------------------------------------*/

/** @defgroup USBPD_CORE_VDM_Exported_Defines USBPD CORE VDM Exported Defines
  * @{
  */
#define MAX_NUM_SVIDS       12
#define MAX_MODES_PER_SVID  6
/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup USBPD_CORE_VDM_Exported_Structures USBPD CORE VDM Exported Structures
  * @{
  */
/**
  * @brief SVDM commands definition
  */
typedef enum
{
  SVDM_RESERVEDCOMMAND          = 0x0,
  SVDM_DISCOVER_IDENTITY        = 0x1,
  SVDM_DISCOVER_SVIDS           = 0x2,
  SVDM_DISCOVER_MODES           = 0x3,
  SVDM_ENTER_MODE               = 0x4,
  SVDM_EXIT_MODE                = 0x5,
  SVDM_ATTENTION                = 0x6,
  SVDM_DP_STATUS                = 0x10,
  SVDM_DP_CONFIG                = 0x11

}USBPD_VDM_Command_Typedef;
  
/** 
  * @brief Product Type field in ID Header
  * @{
  */
typedef enum {
  PRODUCT_TYPE_UNDEFINED     = 0, /*!< Undefined                              */
  PRODUCT_TYPE_HUB           = 1, /*!< PDUSB Hub (UFP)                        */
  PRODUCT_TYPE_PERIPHERAL    = 2, /*!< PDUSB Host (UFP)                       */
#if defined(USBPD_REV30_SUPPORT)
  PRODUCT_TYPE_HOST          = 2, /*!< PDUSB Host  (DFP)                      */
  PRODUCT_TYPE_POWER_BRICK   = 3, /*!< Power Brick (DFP)                      */
#endif /* USBPD_REV30_SUPPORT */
  PRODUCT_TYPE_PASSIVE_CABLE = 3, /*!< Passive Cable (Cable Plug)             */
  PRODUCT_TYPE_ACTIVE_CABLE  = 4, /*!< Active Cable (Cable Plug)              */
#if defined(USBPD_REV30_SUPPORT)
  PRODUCT_TYPE_AMC           = 4, /*!<  Alternate Mode Controller (AMC) (DFP) */
#endif /* USBPD_REV30_SUPPORT */
  PRODUCT_TYPE_AMA           = 5, /*!< Alternate Mode Adapter (AMA) (UFP)     */
} USBPD_ProductType_TypeDef;

/**
  * @}
  */

/**
  * @brief USB Host or Device Capability field in ID Header
  * @{
  */
typedef enum {
  USB_NOTCAPABLE     = 0, /*!< Not USB capable                                     */
  USB_CAPABLE        = 1, /*!< Capable of being enumerated as a USB host or device */
} USBPD_USBCapa_TypeDef;
  
/**
  * @}
  */
  
/**
  * @brief Modal operation field in ID Header
  * @{
  */
typedef enum {
  MODAL_OPERATION_NONSUPP     = 0, /*!< Product not supports Modal Operation. */
  MODAL_OPERATION_SUPPORTED   = 1, /*!< Product supports Modal Operation.     */
} USBPD_ModalOp_TypeDef;
  
/**
  * @}
  */
  
/**
  * @brief Cable to USB field in Active/Passive cable
  * @{
  */
typedef enum {
  CABLE_TO_TYPE_A   = 0, /*!< USB Type-A (PD 2.0 only)  */
  CABLE_TO_TYPE_B   = 1, /*!< USB Type-B (PD 2.0 only)  */
  CABLE_TO_TYPE_C   = 2, /*!< USB Type-C                */
  CABLE_CAPTIVE     = 3, /*!< Captive                   */
} USBPD_CableToType;
  
/**
  * @}
  */
  
/**
  * @brief  cable latency values in nanoseconds (max) in Active/Passive cable
  * @{
  */
typedef enum {
  CABLE_LATENCY_10NS      = 1,  /*!< <10ns (~1m)        */
  CABLE_LATENCY_20NS      = 2,  /*!< 10ns to 20ns (~2m) */
  CABLE_LATENCY_30NS      = 3,  /*!< 20ns to 30ns (~3m) */
  CABLE_LATENCY_40NS      = 4,  /*!< 30ns to 40ns (~4m) */
  CABLE_LATENCY_50NS      = 5,  /*!< 40ns to 50ns (~5m) */
  CABLE_LATENCY_60NS      = 6,  /*!< 50ns to 60ns (~6m) */
  CABLE_LATENCY_70NS      = 7,  /*!< 60ns to 70ns (~7m) */
  CABLE_LATENCY_1000NS    = 8,  /*!< > 70ns (>~7m) for P2.0 or 1000ns  (~100m) for P3.0    */
#if defined(USBPD_REV30_SUPPORT)
  CABLE_LATENCY_2000NS    = 9,  /*!< 2000ns (~200m)     */
  CABLE_LATENCY_3000NS    = 10, /*!< 3000ns (~300m)     */
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableLatency;
  
/** 
  * @}
  */

/** 
  * @brief  Cable Termination Type in Active/Passive cable
  * @{
  */
typedef enum {
  CABLE_TERM_BOTH_PASSIVE_NO_VCONN = 0,  /*!< VCONN not required (PD2.0 only) */
  CABLE_TERM_BOTH_PASSIVE_VCONN    = 1,  /*!< VCONN required (PD2.0 only)     */
#if defined(USBPD_REV30_SUPPORT)
  CABLE_TERM_ONE_EACH_VCONN        = 2,  /*!< One end Active, one end passive, VCONN required */
  CABLE_TERM_BOTH_ACTIVE_VCONN     = 3,  /*!< Both ends Active, VCONN required  */
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableTermType;

/** 
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/** 
  * @brief  Maximum Cable VBUS Voltage in Active/Passive cable
  * @{
  */
typedef enum {
  VBUS_20V            = 0, /*!< Maximum Cable VBUS Voltage 20V */
  VBUS_30V            = 1, /*!< Maximum Cable VBUS Voltage 30V */
  VBUS_40V            = 2, /*!< Maximum Cable VBUS Voltage 40V */
  VBUS_50V            = 3, /*!< Maximum Cable VBUS Voltage 50V */
} USBPD_VBUSMaxVoltage;

/** 
  * @}
  */
#endif /* USBPD_REV30_SUPPORT */

/** 
  * @brief  configurability of SS Directionality in Active/Passive cable and AMA VDO (PD2.0 only)
  * @{
  */
typedef enum {
  SS_DIR_FIXED         = 0, /*!< SSTX Directionality Support Fixed        */
  SS_DIR_CONFIGURABLE  = 1, /*!< SSTX Directionality Support Configurable */
} USBPD_SsDirectionality;

/** 
  * @}
  */

/** 
  * @brief  VBUS Current Handling Capability in Active/Passive cable VDO
  * @{
  */
typedef enum {
  VBUS_3A    = 1, /*!< VBUS  Current Handling Capability 3A */
  VBUS_5A    = 2, /*!< VBUS  Current Handling Capability 5A */
} USBPD_VBUSCurrentHandCap;

/** 
  * @}
  */

/** 
  * @brief  VBUS through cable-ness in Active/Passive cable VDO
  * @{
  */
typedef enum {
  VBUS_THRU_CABLE_NO_ = 0, /*!< No VBUS Through Cable */
  VBUS_THRU_CABLE_YES = 1, /*!< VBUS Through Cable    */
} USBPD_VBUSThruCable;

/** 
  * @}
  */

/** 
  * @brief  SOP" presence in Active cable VDO
  * @{
  */
typedef enum {
  SOP2_NOT_PRESENT = 0, /*!< No SOP" controller present */
  SOP2_PRESENT     = 1, /*!< SOP" controller present    */
} USBPD_Sop2Presence;

/** 
  * @}
  */

/** 
  * @brief  USB Superspeed Signaling Support in Active/Passive cable VDO
  * @{
  */
typedef enum {
  USB2P0_ONLY    = 0, /*!< USB2.0 only*/
  USB3P1_GEN1    = 1, /*!< USB3.1 Gen1 and USB2.0 */
  USB3P1_GEN1N2  = 2, /*!< USB3.1 Gen1, Gen2 and USB2.0*/
} USBPD_UsbSsSupport;

/** 
  * @}
  */

/** 
  * @brief  Power needed by adapter for full functionality in AMA VDO header
  * @{
  */
typedef enum {
  VCONN_1W    = 0, /*!< VCONN  power 1W   */
  VCONN_1P5W  = 1, /*!< VCONN  power 1.5W */
  VCONN_2W    = 2, /*!< VCONN  power 2W   */
  VCONN_3W    = 3, /*!< VCONN  power 3W   */
  VCONN_4W    = 4, /*!< VCONN  power 4W   */
  VCONN_5W    = 5, /*!< VCONN  power 5W   */
  VCONN_6W    = 6, /*!< VCONN  power 6W   */
} USBPD_VConnPower;

/** 
  * @}
  */

/** 
  * @brief  VCONN being required by an adapter in AMA VDO header
  * @{
  */
typedef enum {
  VCONN_NOT_REQUIRED =  0, /*!< VCONN not required  */
  VCONN_REQUIRED     =  1, /*!< VCONN required      */
} USBPD_VConnRequirement;

/** 
  * @}
  */

/** 
  * @brief  VBUS being required by an adapter in AMA VDO header
  * @{
  */
typedef enum {
  VBUS_NOT_REQUIRED = 0, /*!< VBUS not required */
  VBUS_REQUIRED     = 1, /*!< VBUS required     */
} USBPD_VBusRequirement;

/** 
  * @}
  */

/** 
  * @brief  AMA USB Superspeed Signaling Support in AMA VDO header
  * @{
  */
typedef enum {
  AMA_USB2P0_ONLY       = 0, /*!< USB2.0 only                   */
  AMA_USB3P1_GEN1       = 1, /*!< USB3.1 Gen1 and USB2.0        */
  AMA_USB3P1_GEN1N2     = 2, /*!< USB3.1 Gen1, Gen2 and USB2.0  */
  AMA_USB2P0_BILLBOARD  = 3, /*!< USB2.0 billboard only         */
} USBPD_AmaUsbSsSupport;

/** 
  * @}
  */

#if !defined(__PACKED)
  #define __PACKED
#endif

/** @defgroup USBPD_IDHeaderVDOStructure_definition USB SVDM ID header VDO Structure definition
  * @brief USB SVDM ID header VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {                              
    uint16_t VID :16;                         /*!< SVDM Header's SVDM Version                 */
    uint16_t Reserved :10;                    /*!< SVDM Header's reserved bits based on 
                                                   @ref USBPD_IDHeaderVDO_ReservedTypeDef     */
                             
    USBPD_ModalOp_TypeDef ModalOperation :    /*!< Modal Operation Supported */
    1;                           
    USBPD_ProductType_TypeDef ProductType :   /*!< Product Type (UFP or Cable Plug)           */
    3;                            
    USBPD_USBCapa_TypeDef USBDevCapability :  /*!< USB Communications Capable as a USB Device */
    1;       
    USBPD_USBCapa_TypeDef USBHostCapability : /*!< USB Communications Capable as USB Host     */
    1;                     
  }b;
}USBPD_IDHeaderVDO_TypeDef;

typedef union
{
  struct                                        /* PD 2.0               */
  {
    uint16_t Reserved :                         /*!< Reserved           */
    10;                         
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct                                        /* PD 3.0*/
  {
    uint16_t Reserved :                         /*!< Reserved           */
    7;
    USBPD_ProductType_TypeDef ProductTypeDFP :  /*!< Product Type (DFP) */
    3;
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
}USBPD_IDHeaderVDO_ReservedTypeDef;

/**
  * @}
  */

/** @defgroup USBPD_CertStatVdo_TypeDef USB PD VDM Cert stat VDO
  * @brief USB PD Cert stat VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t XID :          /*!< USB-IF assigned XID */
    32;
  }
  b;
}USBPD_CertStatVdo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_ProductVdo_TypeDef USB PD VDM Product VDO
  * @brief USB PD Product VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t bcdDevice :      /*!< Device version             */
    16;
    uint32_t USBProductId :   /*!< USB Product ID             */
    16;
  }
  b;
}USBPD_ProductVdo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_CableVdo_TypeDef USB PD VDM Cable VDO
  * @brief USB PD Cable VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_UsbSsSupport        USB_SS_Support      : 3;  /*!< USB SuperSpeed Signaling Support           */
    USBPD_Sop2Presence        SOP2_Presence       : 1;  /*!< SOP" controller present? (Active cable)    */
    USBPD_VBUSThruCable       VBUS_ThruCable      : 1;  /*!< VBUS through cable                         */
    USBPD_VBUSCurrentHandCap  VBUS_CurrentHandCap : 2;  /*!< VBUS Current Handling Capability           */
    uint8_t Fields1                               : 4;  /*!< Based on @ref USBPD_CableVdo_Field1TypeDef */
    USBPD_CableTermType       CableTermType       : 2;  /*!< Cable Termination Type                     */
    USBPD_CableLatency        CableLatency        : 4;  /*!< Cable Latency                              */
    uint8_t                   Reserved            : 1;  /*!< Reserved                                   */
    USBPD_CableToType         CableToType         : 2;  /*!< USB Type-C plug to USB Type-A/B/C/Captive (PD 2.0) 
                                                             USB Type-C plug to USB Type-C/Captive (PD 3.0) */
    uint8_t Fields2                               : 4;  /*!< Based on @ref USBPD_CableVdo_Field2TypeDef */
    uint8_t                   CableFWVersion      : 4;  /*!< Cable FW version number (vendor defined)   */
    uint8_t                   CableHWVersion      : 4;  /*!< Cable HW version number (vendor defined)   */
  }
  b;
}USBPD_CableVdo_TypeDef;

typedef union
{
  struct /* PD 2.0*/
  {
    USBPD_SsDirectionality    SSRX2_DirSupport    : 1;  /*!< SSRX2 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSRX1_DirSupport    : 1;  /*!< SSRX1 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSTX2_DirSupport    : 1;  /*!< SSTX2 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSTX1_DirSupport    : 1;  /*!< SSTX1 Directionality Support (PD2.0)     */
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct /* PD 3.0*/
  {
    uint8_t                   Reserved            : 2;  /*!< Reserved                                 */
    USBPD_VBUSMaxVoltage      MaxVBUS_Voltage     : 2;  /*!< Maximum Cable VBUS Voltage               */
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableVdo_Field1TypeDef;

typedef union
{
  struct /* PD 2.0*/
  {
    uint8_t                   Reserved            : 4;  /*!< Reserved                               */
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct /* PD 3.0*/
  {
    uint8_t                   VDOVersion          : 3;  /*!< Version Number of the VDO              */
    uint8_t                   Reserved            : 1;  /*!< Reserved                               */
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
}USBPD_CableVdo_Field2TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_AMAVdo_TypeDef USB PD VDM Alternate Mode Adapter VDO
  * @brief USB PD Alternate Mode Adapter VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_AmaUsbSsSupport   AMA_USB_SS_Support  : 3;  /*!< AMA USB SuperSpeed Signaling Support     */
    USBPD_VBusRequirement   VBUSRequirement     : 1;  /*!< VBUS  required                           */
    USBPD_VConnRequirement  VCONNRequirement    : 1;  /*!< VCONN  required                          */
    USBPD_VConnPower        VCONNPower          : 3;  /*!< VCONN  power                             */
    uint16_t Fields                             : 16; /*!< Based on @ref USBPD_AMAVdo_FieldTypeDef  */
    uint8_t                 AMAFWVersion        : 4;  /*!< AMA FW version number (vendor defined)   */
    uint8_t                 AMAHWVersion        : 4;  /*!< AMA HW version number (vendor defined)   */
  }
  b;
}USBPD_AMAVdo_TypeDef;

typedef union
{
  struct /* PD 2.0*/
  {
    USBPD_SsDirectionality  SSRX2_DirSupport    : 1;  /*!< SSRX2 Directionality Support (PD2.0)   */
    USBPD_SsDirectionality  SSRX1_DirSupport    : 1;  /*!< SSRX1 Directionality Support (PD2.0)   */
    USBPD_SsDirectionality  SSTX2_DirSupport    : 1;  /*!< SSTX2 Directionality Support (PD2.0)   */
    USBPD_SsDirectionality  SSTX1_DirSupport    : 1;  /*!< SSTX1 Directionality Support (PD2.0)   */
    uint16_t                Reserved            : 12; /*!< Reserved                               */
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct /* PD 3.0*/
  {
    uint16_t                Reserved            : 13; /*!< Reserved                               */
    uint8_t                 VDO_Version         : 3;  /*!< Version Number of the VDO              */
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
}USBPD_AMAVdo_FieldTypeDef;

/**
  * @}
  */

/** @defgroup USBPD_DiscoveryIdentity_TypeDef USB PD Discovery identity Structure definition
  * @brief Data received from Discover Identity messages
  * @{
  */
typedef struct {

  USBPD_IDHeaderVDO_TypeDef IDHeader;             /*!< ID Header VDO                              */
  USBPD_CertStatVdo_TypeDef CertStatVDO;          /*!< Cert Stat VDO                              */
  USBPD_ProductVdo_TypeDef  ProductVDO;           /*!< Product VDO                                */
  USBPD_CableVdo_TypeDef    CableVDO;             /*!< Passive Cable VDO                          */
  USBPD_AMAVdo_TypeDef      AMA_VDO;              /*!< Active Cable VDO                           */
  uint8_t                   ProductVDO_Presence;  /*!< Indicate Product VDO presence or not       */
  uint8_t                   CableVDO_Presence;    /*!< Indicate Passive Cable VDO presence or not */
  uint8_t                   AMA_VDO_Presence;     /*!< Indicate Active Cable VDO presence or not  */
  uint8_t                   Nack;                 /*!< Information to indicate if Identify is accepted or not */
} USBPD_DiscoveryIdentity_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_SVIDInfo_TypeDef USB PD SVID Info object Structure definition
  * @brief USB PD SVID Info object Structure definition
  * @{
  */
typedef struct {
  uint16_t  SVIDs[MAX_NUM_SVIDS];
  uint8_t   NumSVIDs;
  uint8_t   Nack;
}USBPD_SVIDInfo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_ModeInfo_TypeDef USB PD Mode Info object Structure definition
  * @brief USB PD Mode Info object Structure definition
  * @{
  */
typedef struct {
  uint32_t  NumModes;
  uint32_t  Modes[MAX_MODES_PER_SVID];
  uint16_t  SVID;
  uint8_t   Nack;
}USBPD_ModeInfo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_AttentionInfo_TypeDef USB PD Attention Info object Structure definition
  * @brief USB PD Attention Info object Structure definition
  * @{
  */
typedef struct {
  uint32_t  VDO;
  uint16_t  SVID;
  USBPD_VDM_Command_Typedef Command;
  uint8_t   ModeIndex;
}USBPD_AttentionInfo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_CORE_VDM_Exported_Callback USBPD CORE VDM Exported Callback
  * @{
  */

/**
  * @brief CallBacks exposed by the @ref USBPD_CORE_PHY to the @ref USBPD_CORE_PRL
  * */
typedef struct
{
  /**
    * @brief  VDM Discovery identity callback (answer to Discover Identity message)
    * @param  Port current port number
    * @param  pIdentity Pointer on USBPD_DiscoveryIdentity_TypeDef structure
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_DiscoverIdentity)(uint8_t Port, USBPD_DiscoveryIdentity_TypeDef **pIdentity);

  /**
    * @brief  VDM Discover SVID callback (retrieve SVID supported by device for answer to Discovery mode)
    * @param  Port        current port number
    * @param  p_SVID_Info Pointer on USBPD_SVIDInfo_TypeDef structure
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_DiscoverSVIDs)(uint8_t Port, USBPD_SVIDInfo_TypeDef **p_SVID_Info);

  /**
    * @brief  VDM Discover Mode callback (report all the modes supported by SVID)
    * @param  Port        current port number
    * @param  SVID        SVID ID
    * @param  p_ModeInfo  Pointer on USBPD_ModeInfo_TypeDef structure
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_DiscoverModes)(uint8_t Port, uint16_t SVID, USBPD_ModeInfo_TypeDef **p_ModeInfo);

  /**
    * @brief  VDM Mode enter callback
    * @param  Port      current port number
    * @param  SVID      SVID ID
    * @param  ModeIndex Index of the mode to be entered
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_ModeEnter)(uint8_t Port, uint16_t SVID, uint32_t ModeIndex);

  /**
    * @brief  VDM Mode exit callback
    * @param  Port      current port number
    * @param  SVID      SVID ID
    * @param  ModeIndex Index of the mode to be exited
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_ModeExit)(uint8_t Port, uint16_t SVID, uint32_t ModeIndex);

  /**
    * @brief  Inform identity callback (Identity information received in Discovery identity answer)
    * @param  Port      current port number
    * @param  SOPType   SOP type 
    * @param  Identity  Pointer on the discovery identity information
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformIdentity)(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_DiscoveryIdentity_TypeDef *Identity);

  /**
    * @brief  Inform SVID callback
    * @param  Port current port number
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformSVID)(uint8_t Port, USBPD_SOPType_TypeDef SOPType, uint16_t SVID);

  /**
    * @brief  Inform Mode callback
    * @param  Port current port number
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformMode)(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_ModeInfo_TypeDef *ModesInfo);

  /**
    * @brief  Inform Mode enter callback
    * @param  Port      current port number
    * @param  SVID      SVID ID
    * @param  ModeIndex Index of the mode to be entered
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformModeEnter)(uint8_t Port, uint16_t SVID, uint32_t ModeIndex);

  /**
    * @brief  Inform Mode exit callback
    * @param  Port      current port number
    * @param  SVID      SVID ID
    * @param  ModeIndex Index of the mode to be exited
    * @retval USBPD status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformModeExit)(uint8_t Port, uint16_t SVID, uint32_t ModeIndex);

  /**
    * @brief  Get SVID for Discovery Mode callback
    * @param  Port current port number
    * @retval USBPD status
    */
  uint16_t (*USBPD_VDM_GetSVID)(uint8_t Port);

  /**
    * @brief  Get index of VDO for SVID (Enter mode) callback
    * @param  Port current port number
    * @param  SVID SVID used to be entered
    * @retval Index on the SVID VDO 
    */
  uint32_t (*USBPD_VDM_GetModeIndex)(uint8_t Port, uint16_t SVID);

  /**
    * @brief  VDM Attention callback
    * @param  Port            current port number
    * @param  pAttentionInfo  Pointer on USBPD_AttentionInfo_TypeDef
    * @retval status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_Attention)(uint8_t Port, USBPD_AttentionInfo_TypeDef **pAttentionInfo);

  /**
    * @brief  VDM Inform Attention callback
    * @param  Port            current port number
    * @param  pAttentionInfo  Pointer on USBPD_AttentionInfo_TypeDef
    * @retval status
    */
  USBPD_StatusTypeDef (*USBPD_VDM_InformAttention)(uint8_t Port, USBPD_AttentionInfo_TypeDef *pAttentionInfo);

  /**
    * @brief  VDM Hard Reset callback
    * @param  Port current port number
    * @retval status
    */
  uint32_t (*USBPD_VDM_HardReset)(uint8_t Port);
  
}USBPD_VDM_Callbacks;

/** 
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_CORE_VDM_Exported_Functions
  * @{
  */

/** @addtogroup USBPD_CORE_VDM_Exported_Functions_Group1
  * @{
  */
void USBPD_VDM_TimerCounter(uint8_t PortNum);
/**
  * @}
  */

/** @addtogroup USBPD_CORE_VDM_Exported_Functions_Group2
  * @{
  */
USBPD_StatusTypeDef USBPD_VDM_Init(uint8_t PortNum, USBPD_VDM_Callbacks cbs);
USBPD_StatusTypeDef USBPD_VDM_Reset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_VDM_Process(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_VDM_MessageReceived(uint8_t PortNum, USBPD_SOPType_TypeDef SOPMsgType, uint8_t* RxBuffer, uint8_t NbrVDOs);
USBPD_StatusTypeDef USBPD_SVDM_RequestIdentity(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_SVDM_RequestCableIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_SVDM_RequestModeExit(uint8_t PortNum,USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_SVDM_RequestAttention(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_SVDM_IfEnteredMode(uint8_t PortNum);

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

#endif /* __USBPD_VDM_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
