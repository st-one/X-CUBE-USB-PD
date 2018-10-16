/**
  ******************************************************************************
  * @file    usbpd_vdm_user.c
  * @author  MCD Application Team
  * @brief   USBPD provider demo file
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

/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"
#include "usbpd_vdm_user.h"

/* Private define ------------------------------------------------------------*/
#define SVDM_DP_STATUS SVDM_SPECIFIC_1
#define SVDM_DP_CONFIG SVDM_SPECIFIC_2

#define MAX_SVID_USER   1

/*
 * DP Pin assignement
 */
#define  DP_PIN_ASSIGNMENT_NONE 0x00            /*!< De-select pin assignment.  */
#define  DP_PIN_ASSIGNMENT_A    0x01            /*!< Select Pin Assignment A    */
#define  DP_PIN_ASSIGNMENT_B    0x02            /*!< Select Pin Assignment B    */
#define  DP_PIN_ASSIGNMENT_C    0x04            /*!< Select Pin Assignment C    */
#define  DP_PIN_ASSIGNMENT_D    0x08            /*!< Select Pin Assignment D    */
#define  DP_PIN_ASSIGNMENT_E    0x10            /*!< Select Pin Assignment E    */
#define  DP_PIN_ASSIGNMENT_F    0x20            /*!< Select Pin Assignment F    */

/* Pin configs B/D/F support multi-function */
#define MODE_DP_PIN_MF_MASK 0x2a
/* Pin configs A/B support BR2 signaling levels */
#define MODE_DP_PIN_BR2_MASK 0x3
/* Pin configs C/D/E/F support DP signaling levels */
#define MODE_DP_PIN_DP_MASK 0x3c

#define MODE_DP_V13  0x1
#define MODE_DP_GEN2 0x2

#define MODE_DP_MODE_SNK  0x1
#define MODE_DP_MODE_SRC  0x2
#define MODE_DP_MODE_BOTH 0x3

#define MODE_DP_STATUS_CONNECT_NO     0x0    /*!< no (DFP|UFP)_D is connected or disabled */
#define MODE_DP_STATUS_CONNECT_DFP_D  0x1    /*!< DFP_D connected                         */
#define MODE_DP_STATUS_CONNECT_UFP_D  0x2    /*!< UFP_D connected                         */
#define MODE_DP_STATUS_CONNECT_BOTH   0x3    /*!< DFP_D & UFP_D connected                 */

#define DISPLAY_PORT_SVID       0xFF01U          /*!< SVID For Display Port              */
#define PRODUCT_VDO             0xAAAAAAAAU      /*!< Device version + USB Product ID    */

typedef enum {
  MODE_DP_CONFIG_SELECT_USB,            /*!< Set configuration for USB.           */
  MODE_DP_CONFIG_SELECT_UFP_U_AS_DFP_D, /*!< Set configuration for UFP_U_AS_DFP_. */
  MODE_DP_CONFIG_SELECT_UFP_U_AS_UFP_D, /*!< Set configuration for UFP_U_AS_UFP_D.*/
  MODE_DP_CONFIG_SELECT_RESERVED,       /*!< Reserved                             */
} USBPD_MODE_DP_CONFIG_SELECT;

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
/* GNU Compiler */
#if defined   (__GNUC__)
/* ARM Compiler */
#elif defined   (__CC_ARM)
#pragma anon_unions
/* IAR Compiler */
#elif defined (__ICCARM__)
#endif
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
  };
}USBPD_DPStatus_TypeDef;

/*
 * DisplayPort Config capabilities
 */

typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_MODE_DP_CONFIG_SELECT   SelectConfiguration  : 2;    /*!< Selection configuration */
    uint32_t   Signaling                : 4;    /*!< Signaling for transport of DP protocol */
    uint32_t   Reserved1                : 2;    /*!< Reserved                               */
    uint32_t   UFP_U_Pin                : 8;    /*!< Configure UFP_U pin Assignement        */
    uint32_t   Reserved2                : 16;   /*!< Reserved                               */
  };
}USBPD_DPConfig_TypeDef;

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
  };
}USBPD_DPModeTypeDef;

/*
 * Structure to SVID supported by the devices
 */
typedef struct {
  uint32_t  NumSVIDs;
  uint16_t  SVIDs[MAX_SVID_USER*2%4];
  uint8_t   Nack;
}USBPD_SVIDUSerInfo_TypeDef;

/* Private macro -------------------------------------------------------------*/
/* Private variables  -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* List of callbacks for VDM layer */

static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t PortNum, USBPD_DiscoveryIdentity_TypeDef *pIdentity);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t PortNum, uint16_t **p_SVID_Info, uint8_t *nb);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t PortNum, uint16_t SVID, uint32_t **p_ModeInfo, uint8_t *nbMode);
static USBPD_StatusTypeDef USBPD_VDM_ModeEnter(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef USBPD_VDM_ModeExit(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static void                USBPD_VDM_InformIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef status, USBPD_DiscoveryIdentity_TypeDef *pIdentity);
static USBPD_StatusTypeDef USBPD_VDM_InformSVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_SVIDInfo_TypeDef *pListSVID);
static USBPD_StatusTypeDef USBPD_VDM_InformMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_ModeInfo_TypeDef *pModesInfo);
static USBPD_StatusTypeDef USBPD_VDM_InformModeEnter(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef USBPD_VDM_SendAttention(uint8_t PortNum, uint8_t *NbData, uint32_t *VDO);
static void                USBPD_VDM_ReceiveAttention(uint8_t PortNum, uint8_t NbData, uint32_t VDO);
static USBPD_StatusTypeDef USBPD_VDM_SendSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
static USBPD_StatusTypeDef USBPD_VDM_ReceiveSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
static USBPD_StatusTypeDef USBPD_VDM_InformSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
#ifdef _SNK
static USBPD_StatusTypeDef HPD_Init(uint8_t PortNum);
#endif /* _SNK */

/* Private variables ---------------------------------------------------------*/
const USBPD_VDM_Callbacks vdmCallbacks =
{
  USBPD_VDM_DiscoverIdentity,
  USBPD_VDM_DiscoverSVIDs,
  USBPD_VDM_DiscoverModes,
  USBPD_VDM_ModeEnter,
  USBPD_VDM_ModeExit,
  USBPD_VDM_InformIdentity,
  USBPD_VDM_InformSVID,
  USBPD_VDM_InformMode,
  USBPD_VDM_InformModeEnter,
  NULL,
  USBPD_VDM_SendAttention,
  USBPD_VDM_ReceiveAttention,
  USBPD_VDM_SendSpecific,
  USBPD_VDM_ReceiveSpecific,
  USBPD_VDM_InformSpecific,
};

USBPD_ParamsTypeDef   *pVDM_Params[USBPD_PORT_COUNT];
USBPD_VDM_SettingsTypeDef *pVDM_Settings[USBPD_PORT_COUNT];

uint8_t VDM_Mode_On[USBPD_PORT_COUNT];

USBPD_IDHeaderVDO_TypeDef IDHeaderVDO[USBPD_PORT_COUNT];

USBPD_DiscoveryIdentity_TypeDef sIdentity[USBPD_PORT_COUNT];

USBPD_IDHeaderVDO_TypeDef       Remote_IDHeaderVDO[USBPD_PORT_COUNT];
USBPD_CertStatVdo_TypeDef       Remote_CertStatVDO[USBPD_PORT_COUNT];
USBPD_ProductVdo_TypeDef        Remote_ProductVDO[USBPD_PORT_COUNT];
USBPD_DiscoveryIdentity_TypeDef Remote_sIdentity[USBPD_PORT_COUNT];
uint16_t Remote_CurrentSVID[USBPD_PORT_COUNT];
uint16_t Remote_SVID_Mode[USBPD_PORT_COUNT];

USBPD_SVIDUSerInfo_TypeDef SVIDInfo[USBPD_PORT_COUNT];
USBPD_SVIDPortPartnerInfo_TypeDef SVIDPortPartner[USBPD_PORT_COUNT];
USBPD_ModeInfo_TypeDef sModesInfo[USBPD_PORT_COUNT];

const USBPD_DPModeTypeDef vdo_dp_modes[USBPD_PORT_COUNT][2] =  
{
  {
    {
      .UFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE,              /* UFP pin cfg supported : none         */
      .DFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_E, /* DFP pin cfg supported */
      .USB20            = 1,              /* USB2.0 signalling even in AMode      */
      .PlugOrRecept     = CABLE_TO_PLUG,  /* its a plug                           */
      .Supports         = MODE_DP_V13,    /* DPv1.3 Support, no Gen2              */
      .SignalDirection  = MODE_DP_MODE_SNK  /* Its a sink only                      */
    },
    {
      .UFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE,                  /* UFP pin cfg supported : none     */
      .DFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_D, /* DFP pin cfg supported */
      .USB20            = 0,                  /* USB2.0 signalling even in AMode  */
      .PlugOrRecept     = CABLE_TO_RECEPTACLE,/* its a receptacle                 */
      .Supports         = MODE_DP_V13,        /* DPv1.3 Support, no Gen2          */
      .SignalDirection  = MODE_DP_MODE_BOTH   /* Its a sink / src                 */
    }
  },
};

USBPD_DPStatus_TypeDef sDP_Status[USBPD_PORT_COUNT] = 
{
  {
    .Reserved       = 0,
    .IRQ_HPD        = 0,
    .HPDState       = 0,
    .ExitDPMode     = 0,
    .USBConfig      = 0,
    .MultiFunction  = 1,
    .Enable         = 1,
    .PowerLow       = 0,
    .ConnectStatus  = MODE_DP_STATUS_CONNECT_UFP_D,
  },
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  VDM Discovery identity callback (answer to Discover Identity request message)
  * @param  PortNum current port number
  * @param  pIdentity Pointer on USBPD_DiscoveryIdentity_TypeDef structure
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t PortNum, USBPD_DiscoveryIdentity_TypeDef *pIdentity)
{
  sIdentity[PortNum].Nack                       = 0;
  IDHeaderVDO[PortNum].d32                      = 0;
    IDHeaderVDO[PortNum].b20.VID                    = pVDM_Settings[PortNum]->VDM_USB_VID_SOP;
    IDHeaderVDO[PortNum].b20.ModalOperation         = pVDM_Settings[PortNum]->VDM_ModalOperation;
    IDHeaderVDO[PortNum].b20.USBHostCapability      = pVDM_Settings[PortNum]->VDM_USBHostSupport;
    IDHeaderVDO[PortNum].b20.USBDevCapability       = pVDM_Settings[PortNum]->VDM_USBDeviceSupport;
    IDHeaderVDO[PortNum].b20.ProductTypeUFPorCP     = pVDM_Settings[PortNum]->VDM_ProductTypeUFPorCP;
  sIdentity[PortNum].IDHeader.d32               = IDHeaderVDO[PortNum].d32;
  sIdentity[PortNum].CertStatVDO.b.XID          = pVDM_Settings[PortNum]->VDM_XID_SOP;
  sIdentity[PortNum].ProductVDO.b.USBProductId  = pVDM_Settings[PortNum]->VDM_PID_SOP;
  sIdentity[PortNum].ProductVDO.b.bcdDevice     = pVDM_Settings[PortNum]->VDM_bcdDevice_SOP;

  sIdentity[PortNum].CableVDO_Presence    = 0;
  sIdentity[PortNum].AMA_VDO_Presence     = 0;
  if (PRODUCT_TYPE_AMA == pVDM_Settings[PortNum]->VDM_ProductTypeUFPorCP)
  {
    USBPD_AMAVdo_TypeDef      ama_vdo =
    {
      .b.AMA_USB_SS_Support   = AMA_USB2P0_ONLY,
      .b.VBUSRequirement      = VBUS_REQUIRED,
      .b.VCONNRequirement     = VCONN_NOT_REQUIRED,
      .b.VCONNPower           = VCONN_1W,
      .b.Reserved             = 0x0000,
      .b.AMAFWVersion         = 0x1,
      .b.AMAHWVersion         = 0x1,
    };

    sIdentity[PortNum].AMA_VDO_Presence     = 1;
    sIdentity[PortNum].AMA_VDO.d32          = ama_vdo.d32;
  }


  *pIdentity = sIdentity[PortNum];

  return USBPD_ACK;
}

/**
  * @brief  VDM Discover SVID callback (retrieve SVID supported by device for answer to Discovery mode)
  * @param  PortNum        current port number
  * @param  p_SVID_Info Pointer on USBPD_SVIDInfo_TypeDef structure
  * @retval USBPD status USBPD_ACK, USBPD_NAK, USBPD_BUSY
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t PortNum, uint16_t **p_SVID_Info, uint8_t *nb)
{
  *nb = SVIDInfo[PortNum].NumSVIDs;
  *p_SVID_Info = SVIDInfo[PortNum].SVIDs;
  return USBPD_ACK;  
}

/**
  * @brief  VDM Discover Mode callback (report all the modes supported by SVID)
  * @param  PortNum current port number
  * @param  SVID SVID ID
  * @param  p_ModeInfo Pointer on USBPD_ModeInfo_TypeDef structure
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t PortNum, uint16_t SVID, uint32_t **p_ModeInfo, uint8_t *nbMode)
{
  USBPD_StatusTypeDef _status = USBPD_NAK;
  
  switch(SVID)
  {
  case DISPLAY_PORT_SVID :
    *nbMode  = 2;
    *p_ModeInfo = (uint32_t *)vdo_dp_modes;
    _status = USBPD_ACK;
    break;
  default :
    *nbMode = 0;
    *p_ModeInfo = NULL;
     break;
  }
  return _status;
}
/**
  * @brief  VDM Mode enter callback
  * @param  PortNum current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_ModeEnter(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_NAK;
  
  if(VDM_Mode_On[PortNum] != 0) return status; 
  
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 1;
  }
  
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 2;
  }
  
  return status;
}

/**
  * @brief  VDM Mode exit callback
  * @param  PortNum current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be exited
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_ModeExit(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_NAK;

  if(VDM_Mode_On[PortNum] == 0) return status; 
  
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1) && (VDM_Mode_On[PortNum] == 1))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 0;
  }

  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2) && (VDM_Mode_On[PortNum] == 2))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 0;
  }

  return status;
}
/**
  * @brief  VDM Send Specific message callback
  * @param  PortNum    current port number
  * @param  NbData     Pointer of number of VDO to send
  * @param  VDO        Pointer of VDO to send
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_SendAttention(uint8_t PortNum, uint8_t *NbData, uint32_t *VDO)
{
  USBPD_StatusTypeDef status = USBPD_ACK;
  if (USBPD_PORTDATAROLE_DFP == pVDM_Params[PortNum]->PE_DataRole)
  {
    USBPD_DPStatus_TypeDef dp_status = {0};
    dp_status.ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;
    
    *VDO = dp_status.d32;
    *NbData = 1;
  }
#if _SNK_MUX
  else
  {
    sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;
    
    *VDO = sDP_Status[PortNum].d32;
    *NbData = 1;
  }
#endif /*_SNK_MUX*/
  return status;
}

/**
  * @brief  VDM Attention callback to forward information from PE stack
  * @param  PortNum   current port number
  * @param  NbData    Number of received VDO
  * @param  VDO       Received VDO
  * @retval status
  */
static void USBPD_VDM_ReceiveAttention(uint8_t PortNum, uint8_t NbData, uint32_t VDO)
{
  if (USBPD_PORTDATAROLE_DFP == pVDM_Params[PortNum]->PE_DataRole)
  {
    if (NbData == 1)
    {
      USBPD_DPStatus_TypeDef vdo;
      vdo.d32 = VDO;
      
      /* Check state of HPD pin in attention message */
      if (0 == vdo.HPDState)
      {
#ifdef _SRC_MUX
        /* Screen has been disconnected... Disconnect laptop */
        BSP_MUX_SetHPDState(TYPE_C_MUX_1, HPD_STATE_LOW);
#endif /*_SRC_MUX*/
      }
      else
      {
        /* Screen has been connected... Sent a config message to screen for connection*/
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
      }
    }
  }
  else
  {
    if (NbData == 1)
    {
      /* Check if DP Status has been received in the attention message*/
      USBPD_DPStatus_TypeDef dp_status;
      dp_status.d32 = VDO;
    
      if (dp_status.ConnectStatus == MODE_DP_STATUS_CONNECT_UFP_D)
      {
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
      }
    }
  }
}

/**
  * @brief  VDM Discovery identity callback (answer after sending a Discover Identity req message)
  * @param  PortNum       current port number
  * @param  SOPType       SOP type 
  * @param  CommandStatus Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pIdentity     Pointer on the discovery identity information based on @ref USBPD_DiscoveryIdentity_TypeDef
  * @retval None
*/
static void USBPD_VDM_InformIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_DiscoveryIdentity_TypeDef *pIdentity)
{
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    Remote_IDHeaderVDO[PortNum].d32  = pIdentity->IDHeader.d32;
    Remote_CertStatVDO[PortNum].d32  = pIdentity->CertStatVDO.d32;
    Remote_ProductVDO[PortNum].d32   = pIdentity->ProductVDO.d32;

    {
      /* Alternate mode presence */
      if (pIdentity->AMA_VDO_Presence == 1)
      {
        /* Request to get SVID */
        USBPD_PE_SVDM_RequestSVID(PortNum, USBPD_SOPTYPE_SOP);
      }
    }
    SVIDPortPartner[PortNum].NumSVIDs = 0;
    Remote_CurrentSVID[PortNum]       = 0;
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    // retry in 50ms 
    break;
  default :
    break;
  }
}

/**
  * @brief  Inform SVID callback
  * @param  PortNum       current port number
  * @param  SOPType       SOP type 
  * @param  CommandStatus Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pListSVID     Pointer of list of SVID based on @ref USBPD_SVIDInfo_TypeDef
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformSVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_SVIDInfo_TypeDef *pListSVID)
{
  uint32_t index = 0;

  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    for (index = 0; index < pListSVID->NumSVIDs; index++)
    {
      SVIDPortPartner[PortNum].SVIDs[SVIDPortPartner[PortNum].NumSVIDs++] = pListSVID->SVIDs[index];
    }

    /* Check if all the SVIDs have been received */
    if (pListSVID->AllSVID_Received == 0)
    {
      /* Request a new SVID message */
      USBPD_PE_SVDM_RequestSVID(PortNum, USBPD_SOPTYPE_SOP);
    }
    else
    {
      /* All the SVIDs have been received, request discovery mode on 1st SVID available
      in the list */
      /* Request a discovery mode */
      Remote_SVID_Mode[PortNum] = 0;
      {
        USBPD_PE_SVDM_RequestMode(PortNum, USBPD_SOPTYPE_SOP, SVIDPortPartner[PortNum].SVIDs[0]);
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    // retry in 50ms 
    break;
  default :
    break;
  }

  return USBPD_OK;
}

/**
  * @brief  Inform Mode callback
  * @param  PortNum         current port number
  * @param  SOPType         SOP type 
  * @param  CommandStatus   Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pModesInfo      Pointer of Modes info based on @ref USBPD_ModeInfo_TypeDef
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_ModeInfo_TypeDef *pModesInfo)
{
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    {
      USBPD_DPModeTypeDef dp_mode;
      sModesInfo[PortNum].NumModes = pModesInfo->NumModes;
      for (uint32_t index = 0; index < sModesInfo[PortNum].NumModes; index++)
      {
        sModesInfo[PortNum].Modes[index] = pModesInfo->Modes[index];
      }

      sModesInfo[PortNum].SVID = pModesInfo->SVID;
      sModesInfo[PortNum].Nack = pModesInfo->Nack;

      dp_mode.d32 = (sModesInfo[PortNum].Modes[0]);

      {
        /* Enter in the mode only if DFP_D and UFP_D are supported */
        if ((dp_mode.SignalDirection == MODE_DP_MODE_BOTH) || (MODE_DP_MODE_SNK == dp_mode.SignalDirection))
        {
          /* Request to enter in the 1st mode */
          USBPD_PE_SVDM_RequestModeEnter(PortNum, SOPType, pModesInfo->SVID, 1);
        }
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    {
      /* All the SVIDs have been received, request discovery mode on next SVID available
      in the list */
      Remote_SVID_Mode[PortNum]++;
      /* Request a discovery mode */
      USBPD_PE_SVDM_RequestMode(PortNum, USBPD_SOPTYPE_SOP, SVIDPortPartner[PortNum].SVIDs[Remote_SVID_Mode[PortNum]]);
    }
    break;
  case SVDM_RESPONDER_BUSY :
    // retry in 50ms 
    break;
  default :
    break;
  }
  
  return USBPD_OK;
}

/**
  * @brief  Inform Mode enter callback
  * @param  PortNum   current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformModeEnter(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    {
      if (SVIDPortPartner[PortNum].SVIDs[(ModeIndex - 1)] == SVID)
      {
        status = USBPD_OK;
        VDM_Mode_On[PortNum] = 1;
        
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_STATUS, DISPLAY_PORT_SVID);
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    status = USBPD_OK;
    break;
  case SVDM_RESPONDER_BUSY :
    // retry in 50ms 
    break;
  default :
    break;
  }
  
  return status;
}

/**
  * @brief  VDM Send Specific message callback
  * @param  PortNum            current port number
  * @param  VDMCommand VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  NbData     Pointer of number of VDO to send
  * @param  VDO        Pointer of VDO to send
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_SendSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO)
{
  USBPD_StatusTypeDef status = USBPD_ACK;
  if (USBPD_PORTDATAROLE_DFP == pVDM_Params[PortNum]->PE_DataRole)
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef dp_status = {0};
        dp_status.ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;

        *VDO = dp_status.d32;
        *NbData = 1;
      }
      break;
    case SVDM_DP_CONFIG :
      {
        USBPD_DPModeTypeDef dp_mode;
        dp_mode.d32 = (sModesInfo[PortNum].Modes[0]);
        
        if (DP_PIN_ASSIGNMENT_C == (dp_mode.DFP_D_Pin & DP_PIN_ASSIGNMENT_C))
        {
          USBPD_DPConfig_TypeDef vdo_config = {0};
          vdo_config.SelectConfiguration = MODE_DP_CONFIG_SELECT_UFP_U_AS_DFP_D;
          /* Support of DPv1.3 */
          vdo_config.Signaling = 0x1;
          vdo_config.UFP_U_Pin = DP_PIN_ASSIGNMENT_C;
          *VDO = vdo_config.d32;
          *NbData = 1;
        }
        else
        {
          /* Config not supported */
          status = USBPD_NAK;
        }
      }
      break;
    default :
      status = USBPD_NAK;
      break;
    }
  }
#if _SNK_MUX
  else
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;

        *VDO = sDP_Status[PortNum].d32;
        *NbData = 1;    
      }
      break;
    case SVDM_DP_CONFIG :
      *VDO = 0;
      *NbData = 0;    
      break;
      
    default :
      status = USBPD_NAK;
      break;
    }
  }
#endif /*_SNK_MUX*/
  return status;
}

/**
  * @brief  VDM Receive Specific message callback
  * @param  PortNum     Current port number
  * @param  VDMCommand  VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  pNbData     Pointer of number of received VDO and used for the answer
  * @param  pVDO        Pointer of received VDO and use for the answer
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_ReceiveSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *pNbData, uint32_t *pVDO)
{
  USBPD_StatusTypeDef status = USBPD_ACK;
  if (USBPD_PORTDATAROLE_DFP == pVDM_Params[PortNum]->PE_DataRole)
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        /* Port0 is configured as SRC */
        sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;
        
        *pVDO = sDP_Status[PortNum].d32;
        *pNbData = 1;
      }
      break;
    case SVDM_DP_CONFIG :
      break;
      
    default :
      status = USBPD_NAK;
      break;
    }
  }
  else
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo;
        vdo.d32 = 0;
        if (*pNbData == 1)
        {
          vdo.d32 = pVDO[0];
        }
        if ((MODE_DP_STATUS_CONNECT_DFP_D == vdo.ConnectStatus) || (MODE_DP_STATUS_CONNECT_BOTH == vdo.ConnectStatus))
        {
          sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;
          
          *pVDO = sDP_Status[PortNum].d32;
          *pNbData = 1;
        }
        else
        {
          status = USBPD_NAK;
        }
      }
      break;
    case SVDM_DP_CONFIG :
      {
#if _SNK_MUX
        USBPD_DPConfig_TypeDef vdo;
        vdo.d32 = pVDO[0];
        
        if (USBPD_PORT_1 == PortNum)
        {
          MUX_TypeCConnectorPinAssignmentTypeDef assignement = UFP_PIN_ASSIGNMMENT_A;
          switch(vdo.UFP_U_Pin)
          {
          case DP_PIN_ASSIGNMENT_NONE:         /*!< De-select pin assignment.  */
            assignement = USB_ONLY_PIN_ASSIGNMMENT;
            break;
          case DP_PIN_ASSIGNMENT_A:            /*!< Select Pin Assignment A    */
            assignement = UFP_PIN_ASSIGNMMENT_A;
            break;
          case DP_PIN_ASSIGNMENT_B:            /*!< Select Pin Assignment B    */
            assignement = UFP_PIN_ASSIGNMMENT_B;
            break;
          case DP_PIN_ASSIGNMENT_C:            /*!< Select Pin Assignment C    */
            assignement = UFP_PIN_ASSIGNMMENT_C;
            break;
          case DP_PIN_ASSIGNMENT_D:            /*!< Select Pin Assignment D    */
            assignement = UFP_PIN_ASSIGNMMENT_D;
            break;
          case DP_PIN_ASSIGNMENT_E:            /*!< Select Pin Assignment E    */
            assignement = UFP_PIN_ASSIGNMMENT_E;
            break;
          case DP_PIN_ASSIGNMENT_F:            /*!< Select Pin Assignment F    */
            assignement = UFP_PIN_ASSIGNMMENT_F;
            break;
          default :
            status = USBPD_NAK;
            break;
          }
          if (CC1 == pVDM_Params[USBPD_PORT_1]->ActiveCCIs)
            BSP_MUX_SetDPPinAssignment(TYPE_C_MUX_2, PLUG_ORIENTATION_NORMAL, assignement);
          else
            BSP_MUX_SetDPPinAssignment(TYPE_C_MUX_2, PLUG_ORIENTATION_FLIPPED, assignement);
        }
#endif /*_SNK_MUX*/
        *pVDO = 0;
        *pNbData = 0;
      }
      break;
      
    default :
      status = USBPD_NAK;
      break;
    }
  }
  return status;
}

/**
  * @brief  VDM Inform Specific callback
  * @param  PortNum           current port number
  * @param  VDMCommand VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  NbData     Pointer of number of received VDO
  * @param  VDO        Pointer of received VDO
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO)
{
  if (USBPD_PORTDATAROLE_DFP == pVDM_Params[PortNum]->PE_DataRole)
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo_received;
        vdo_received.d32 = VDO[0];
        
        if (1 == vdo_received.HPDState)
        {
          USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
        }
      }
      break;
    case SVDM_DP_CONFIG :
#if _SRC_MUX
      if (MUX_OK != BSP_MUX_SetHPDState(TYPE_C_MUX_1, HPD_STATE_HIGH))
      {
        return USBPD_ERROR;
      }
      
      if (CC1 == pVDM_Params[USBPD_PORT_0]->ActiveCCIs)
        BSP_MUX_SetDPPinAssignment(TYPE_C_MUX_1, PLUG_ORIENTATION_NORMAL, DFP_PIN_ASSIGNMMENT_C);
      else
        BSP_MUX_SetDPPinAssignment(TYPE_C_MUX_1, PLUG_ORIENTATION_FLIPPED, DFP_PIN_ASSIGNMMENT_C);
#endif /*_SRC_MUX*/
      break;
      
    default :
      return USBPD_NAK;
    }
  }
  else
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo_received;

        vdo_received.d32 = VDO[0];

        if (vdo_received.ConnectStatus != MODE_DP_STATUS_CONNECT_NO)
        {
          sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;
        }
        else
        {
          /* DP not connected */
          sDP_Status[PortNum].ConnectStatus = MODE_DP_STATUS_CONNECT_NO;
        }
      }
      break;
    case SVDM_DP_CONFIG :
      break;

    default :
      return USBPD_NAK;
    }
  }
  return USBPD_ACK;
}

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  VDM Initialization function
  * @param  PortNum     Index of current used port
  * @param  pSettings   Pointer on @ref USBPD_VDM_SettingsTypeDef structure
  * @param  pParams     Pointer on @ref USBPD_ParamsTypeDef structure
  * @retval status
  */
USBPD_StatusTypeDef USBPD_VDM_UserInit(uint8_t PortNum, USBPD_VDM_SettingsTypeDef *pSettings, USBPD_ParamsTypeDef *pParams)
{
  uint32_t index = 0;
  
  pVDM_Params[PortNum] = pParams;
  pVDM_Settings[PortNum] = pSettings;

   /* Initialize SVID supported by consumer */
  SVIDInfo[PortNum].NumSVIDs = MAX_SVID_USER;
  
  for (index = 0; index < MAX_SVID_USER; index++)
  {
    SVIDInfo[PortNum].SVIDs[index] = DISPLAY_PORT_SVID + index;
  }

  USBPD_PE_InitVDM_Callback(PortNum, (USBPD_VDM_Callbacks *)&vdmCallbacks);

#ifdef _SNK
  if (USBPD_PORT_1 == PortNum)
    return HPD_Init(PortNum);
  else
    return USBPD_OK;
#else
  return USBPD_OK;
#endif /* _SNK */
}

/**
  * @brief  VDM Reset function
  * @retval status
  */
void USBPD_VDM_UserReset(uint8_t PortNum)
{
   VDM_Mode_On[PortNum] = 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
