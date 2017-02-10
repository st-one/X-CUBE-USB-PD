/**
  ******************************************************************************
  * @file    usbpd_vdm_callback.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   USBPD provider demo file
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

/* Includes ------------------------------------------------------------------*/
#include "usbpd_vdm_callbacks.h"
#include "usbpd_vdm.h"
#include "usbpd_pe.h"

/* Private define ------------------------------------------------------------*/
#define MAX_SVID_USER   1

/* Pin assignment */
#define MODE_DP_PIN_A 0x01
#define MODE_DP_PIN_B 0x02
#define MODE_DP_PIN_C 0x04
#define MODE_DP_PIN_D 0x08
#define MODE_DP_PIN_E 0x10
#define MODE_DP_PIN_F 0x20

/* Pin configs B/D/F support multi-function */
#define MODE_DP_PIN_MF_MASK 0x2a
/* Pin configs A/B support BR2 signaling levels */
#define MODE_DP_PIN_BR2_MASK 0x3
/* Pin configs C/D/E/F support DP signaling levels */
#define MODE_DP_PIN_DP_MASK 0x3c

#define MODE_DP_V13  0x1
#define MODE_DP_GEN2 0x2

#define MODE_DP_SNK  0x1
#define MODE_DP_SRC  0x2
#define MODE_DP_BOTH 0x3

#define ST_VID                  0x0483U          /*!< USB-IF ID for ST-Microelectronics  */
#define DISPLAY_PORT_SVID       0xFF01U          /*!< SVID For Display Port              */
#define CERTSTATVDO             0x0000AAAAU      /*!< USB-IF assigned XID                */
#define PRODUCT_VDO             0xAAAAAAAAU      /*!< Device version + USB Product ID    */

/** 
  * @brief Type-C to Plug/Receptacle
  * @{
  */
typedef enum {
  CABLE_TO_PLUG        = 0, /*0b0*/
  CABLE_TO_RECEPTACLE  = 1, /*0b1*/
} USBPD_CableToPR;

/** 
  * @}
  */

/* Private typedef -----------------------------------------------------------*/
/*
 * DisplayPort Status VDO
 */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t   ConnectStatus  : 2;    /*!< Connect status : 00b ==  no (DFP|UFP)_D is connected or disabled.  
                                           01b == DFP_D connected, 10b == UFP_D connected, 11b == both        */
    uint32_t   PowerLow       : 1;    /*!< Power low : 0 == normal or LPM disabled, 1 == DP disabled for LPM  */
    uint32_t   Enable         : 1;    /*!< Enabled : is DPout on/off.                                         */
    uint32_t   MultiFunction  : 1;    /*!< Multi-function preference : 0 == no pref, 1 == MF preferred        */
    uint32_t   USBConfig      : 1;    /*!< USB config : 0 == maintain current, 1 == switch to USB from DP     */
    uint32_t   ExitDPMode     : 1;    /*!< Exit DP Alt mode: 0 == maintain, 1 == exit                         */
    uint32_t   HPDState       : 1;    /*!< HPD state : 0 = HPD_LOW, 1 == HPD_HIGH                             */
    uint32_t   IRQ_HPD        : 1;    /*!< IRQ_HPD : 1 == irq arrived since last message otherwise 0.         */
    uint32_t   Reserved       : 22;   /*!< Reserved                                                           */
  }
  b;
}USBPD_DPStatus_TypeDef;

/*
 * DisplayPort modes capabilities
 */

typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t   SignalDirection  : 2;    /*!< signal direction ( 00b=rsv, 01b=sink, 10b=src 11b=both ) */
    uint32_t   Supports         : 4;    /*!< xxx1: Supports DPv1.3, xx1x Supports USB Gen 2 signaling
                                             Other bits are reserved.                                 */
    uint32_t   PlugOrRecept     : 1;    /*!< Plug | Receptacle (0b == plug, 1b == receptacle)         */
    uint32_t   USB20            : 1;    /*!< USB 2.0 signaling (0b=yes, 1b=no)                        */
    uint32_t   DFP_D_Pin        : 8;    /*!< DFP_D pin assignment supported                           */
    uint32_t   UFP_D_Pin        : 8;    /*!< UFP_D pin assignment supported                           */
    uint32_t   Reserved         : 8;    /*!< Reserved                                                 */
  }
  b;
}USBPD_DPModeTypeDef;

/*
 * Structure to SVID supported by the devices
 */
typedef struct {
  uint32_t  NumSVIDs;
  uint16_t  SVIDs[MAX_SVID_USER];
  uint8_t   Nack;
}USBPD_SVIDUSerInfo_TypeDef;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* List of callbacks for VDM layer */

static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t Port, USBPD_DiscoveryIdentity_TypeDef **pIdentity);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t Port, USBPD_SVIDInfo_TypeDef **p_SVID_Info);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t Port, uint16_t SVID, USBPD_ModeInfo_TypeDef **p_ModeInfo);
static USBPD_StatusTypeDef USBPD_VDM_EnterMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef USBPD_VDM_ExitMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef USBPD_VDM_Attention(uint8_t Port, USBPD_AttentionInfo_TypeDef **pAttentionInfo);
static USBPD_StatusTypeDef USBPD_VDM_InformAttention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef *pAttentionInfo);

/* Private variables ---------------------------------------------------------*/
USBPD_VDM_Callbacks vdmCallbacks =
{
  USBPD_VDM_DiscoverIdentity,
  USBPD_VDM_DiscoverSVIDs,
  USBPD_VDM_DiscoverModes,
  USBPD_VDM_EnterMode,
  USBPD_VDM_ExitMode,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  USBPD_VDM_Attention,
  USBPD_VDM_InformAttention,
  NULL,
};
uint8_t VDM_Mode_On = 0;

USBPD_IDHeaderVDO_TypeDef IDHeaderVDO = 
{  
  .b.USBHostCapability = USB_NOTCAPABLE,
  .b.USBDevCapability  = USB_NOTCAPABLE,
  .b.ProductType       = PRODUCT_TYPE_UNDEFINED,  
  .b.ModalOperation    = MODAL_OPERATION_SUPPORTED,           
  .b.Reserved          = 0,              
  .b.VID               = ST_VID,
};

USBPD_DiscoveryIdentity_TypeDef sIdentity;
USBPD_SVIDUSerInfo_TypeDef SVIDInfo;
USBPD_AttentionInfo_TypeDef sAttention;
uint32_t CurrentSVID_Index = 0;

USBPD_ModeInfo_TypeDef mode_info;
USBPD_SVIDInfo_TypeDef sSVID_Info;

const USBPD_DPModeTypeDef vdo_dp_modes[2] =  
{
  {
    .b.UFP_D_Pin        = 0,              /* UFP pin cfg supported : none         */
    .b.DFP_D_Pin        = MODE_DP_PIN_C | MODE_DP_PIN_D, /* DFP pin cfg supported */
    .b.USB20            = 0,              /* USB2.0 signalling even in AMode      */
    .b.PlugOrRecept     = CABLE_TO_PLUG,  /* its a plug                           */
    .b.Supports         = MODE_DP_V13,    /* DPv1.3 Support, no Gen2              */
    .b.SignalDirection  = MODE_DP_SNK     /* Its a sink only                      */
  },
  {
    .b.UFP_D_Pin        = 0,                  /* UFP pin cfg supported : none     */
    .b.DFP_D_Pin        = MODE_DP_PIN_C | MODE_DP_PIN_D, /* DFP pin cfg supported */
    .b.USB20            = 0,                  /* USB2.0 signalling even in AMode  */
    .b.PlugOrRecept     = CABLE_TO_RECEPTACLE,/* its a receptacle                 */
    .b.Supports         = MODE_DP_V13,        /* DPv1.3 Support, no Gen2          */
    .b.SignalDirection  = MODE_DP_BOTH        /* Its a sink / src                 */
  }
};

USBPD_DPStatus_TypeDef sDP_Status = 
{
  .b.Reserved       = 0,
  .b.IRQ_HPD        = 0,
  .b.HPDState       = 0,
  .b.ExitDPMode     = 0,
  .b.USBConfig      = 0,
  .b.MultiFunction  = 0,
  .b.Enable         = 1,
  .b.PowerLow       = 0,
  .b.ConnectStatus  = 2,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  VDM Discovery identity callback (answer to Discover Identity message)
  * @param  Port current port number
  * @param  pIdentity Pointer on USBPD_DiscoveryIdentity_TypeDef structure
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t Port, USBPD_DiscoveryIdentity_TypeDef **pIdentity)
{
  *pIdentity = &sIdentity;
  
  sIdentity.Nack                 = 0;
  sIdentity.IDHeader.d32         = IDHeaderVDO.d32;
  sIdentity.CertStatVDO.d32      = CERTSTATVDO;
  sIdentity.ProductVDO_Presence  = 1;
  sIdentity.ProductVDO.d32       = PRODUCT_VDO;

  sIdentity.CableVDO_Presence    = 0;
  sIdentity.AMA_VDO_Presence     = 0;
  CurrentSVID_Index = 0;
  
  return USBPD_OK;
}

/**
  * @brief  VDM Discover SVID callback (retrieve SVID supported by device for answer to Discovery mode)
  * @param  Port        current port number
  * @param  p_SVID_Info Pointer on USBPD_SVIDInfo_TypeDef structure
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t Port, USBPD_SVIDInfo_TypeDef **p_SVID_Info)
{
  uint32_t index = 0;
  
  *p_SVID_Info = &sSVID_Info;
  
  sSVID_Info.SVIDs[0] = 0;
  sSVID_Info.NumSVIDs = 0;
  
  for (index = 0; (index < MAX_NUM_SVIDS && CurrentSVID_Index < SVIDInfo.NumSVIDs); index++, CurrentSVID_Index++)
  {
    sSVID_Info.SVIDs[index] = SVIDInfo.SVIDs[CurrentSVID_Index];
  }
  if (MAX_NUM_SVIDS == index)
  {
    sSVID_Info.NumSVIDs = MAX_NUM_SVIDS;
  }
  else
  {
    sSVID_Info.NumSVIDs = index;
  }
  
  return USBPD_OK;  
}

/**
  * @brief  VDM Discover Mode callback (report all the modes supported by SVID)
  * @param  Port current port number
  * @param  SVID SVID ID
  * @param  p_ModeInfo Pointer on USBPD_ModeInfo_TypeDef structure
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t Port, uint16_t SVID, USBPD_ModeInfo_TypeDef **p_ModeInfo)
{
  *p_ModeInfo = &mode_info;
  
  if (DISPLAY_PORT_SVID == SVID)
  {
    mode_info.Nack      = 0;
    mode_info.SVID      = SVID;
    mode_info.NumModes  = 2;
    mode_info.Modes[0]  = vdo_dp_modes[0].d32;
    mode_info.Modes[1]  = vdo_dp_modes[1].d32;
  }
  else
  {
    mode_info.Nack      = 1;
    mode_info.SVID      = SVID;
    mode_info.NumModes  = 0;
    mode_info.Modes[0]  = 0;
  }
  
  return USBPD_OK;
}

/**
  * @brief  VDM Mode enter callback
  * @param  Port current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_EnterMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1))
  {
    status = USBPD_OK;
    VDM_Mode_On = 1;
  }
  
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2))
  {
    status = USBPD_OK;
    VDM_Mode_On = 1;
  }
  
  return status;
}

/**
  * @brief  VDM Mode exit callback
  * @param  Port current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be exited
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_ExitMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;

  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1))
  {
    status = USBPD_OK;
    VDM_Mode_On = 2;
  }

  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2))
  {
    status = USBPD_OK;
    VDM_Mode_On = 2;
  }

  return status;
}

/**
  * @brief  VDM Attention callback
  * @param  Port current port number
  * @param  pAttentionInfo Pointer on USBPD_AttentionInfo_TypeDef
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_Attention(uint8_t Port, USBPD_AttentionInfo_TypeDef **pAttentionInfo)
{
  *pAttentionInfo = &sAttention;
  if (sAttention.Command == SVDM_DP_STATUS)
   sAttention.VDO        = sDP_Status.d32;

  return USBPD_OK;
}

/**
  * @brief  VDM Inform Attention callback
  * @param  Port current port number
  * @param  pAttentionInfo Pointer on USBPD_AttentionInfo_TypeDef
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformAttention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef *pAttentionInfo)
{
  USBPD_DPStatus_TypeDef vdo_received;
  vdo_received.d32 = pAttentionInfo->VDO;
  
  if (vdo_received.b.ConnectStatus != 0)
  {
    sDP_Status.b.ConnectStatus = 2;
    sAttention.VDO        = sDP_Status.d32;
    sAttention.Command    = SVDM_DP_STATUS;
  }
  else
  {
    /* DP not connected */
      sDP_Status.b.ConnectStatus = 0;
  }

  return USBPD_OK;
}

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  VDM Initialization function
  * @retval status
  */
USBPD_StatusTypeDef VDM_Initialization(void)
{
  /* Initialize SVID supported by consumer */
  SVIDInfo.NumSVIDs = MAX_SVID_USER;
  
  for (uint32_t index = 0; index < MAX_SVID_USER; index++)
  {
    SVIDInfo.SVIDs[index] = DISPLAY_PORT_SVID + index;
  }
  
  USBPD_VDM_Init(0, vdmCallbacks);
  return USBPD_OK;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
