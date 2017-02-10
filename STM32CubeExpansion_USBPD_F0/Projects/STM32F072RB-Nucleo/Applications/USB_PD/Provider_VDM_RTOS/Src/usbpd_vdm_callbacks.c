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
 * DisplayPort Configure VDO
 */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t   Config           : 2;    /*!< cfg : 00 == USB, 01 == DFP_D, 10 == UFP_D, 11 == reserved  */
    uint32_t   Signalling       : 4;    /*!< signalling : 1h == DP v1.3, 2h == Gen 2 
                                             Oh is only for USB, remaining values are reserved          */
    uint32_t   Reserved1        : 2;    /*!< Reserved   */
    uint32_t   PinAssign        : 8;    /*!< Pin assignment requested.  Choose one from mode caps.      */
    uint32_t   Reserved2        : 8;    /*!< Reserved                                                   */
    uint32_t   Reserved3        : 8;    /*!< Reserved                                                   */
  }
  b;
}USBPD_DPConfigTypeDef;

/*
 * Structure to SVID retrieved for Discovery SVID
 */
typedef struct {
  uint32_t  NumSVIDs;
  uint16_t  SVIDs[100];
  uint8_t   Nack;
}USBPD_SVIDUSerInfo_TypeDef;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* List of callbacks for VDM layer */
static USBPD_StatusTypeDef  USBPD_VDM_InformIdentity(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_DiscoveryIdentity_TypeDef *Identity);
static USBPD_StatusTypeDef  USBPD_VDM_InformSVID(uint8_t Port, USBPD_SOPType_TypeDef SOPType, uint16_t SVID);
static uint16_t             USBPD_VDM_GetSVID(uint8_t Port);
static uint32_t             USBPD_VDM_GetModeIndex(uint8_t Port, uint16_t SVID);
static USBPD_StatusTypeDef  USBPD_VDM_InformMode(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_ModeInfo_TypeDef *ModesInfo);

static USBPD_StatusTypeDef  USBPD_VDM_EnterMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef  USBPD_VDM_InformEnterMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef  USBPD_VDM_InformExitMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef  USBPD_VDM_Attention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef **pAttentionInfo);
static USBPD_StatusTypeDef  USBPD_VDM_InformAttention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef *pAttentionInfo);

/* Private variables ---------------------------------------------------------*/
USBPD_VDM_Callbacks vdmCallbacks =
{
  NULL,
  NULL,
  NULL,
  USBPD_VDM_EnterMode,
  NULL,
  USBPD_VDM_InformIdentity,
  USBPD_VDM_InformSVID,
  USBPD_VDM_InformMode,
  USBPD_VDM_InformEnterMode,
  USBPD_VDM_InformExitMode,
  USBPD_VDM_GetSVID,
  USBPD_VDM_GetModeIndex,
  USBPD_VDM_Attention,
  USBPD_VDM_InformAttention,
  NULL,
};
uint8_t VDM_Mode_On = 0;


USBPD_IDHeaderVDO_TypeDef IDHeaderVDO;
USBPD_SVIDUSerInfo_TypeDef SVIDInfo;
USBPD_ModeInfo_TypeDef sModesInfo;
USBPD_AttentionInfo_TypeDef sAttention;
USBPD_DPModeTypeDef sDP_Mode = {0};
USBPD_DPConfigTypeDef sDP_Config = 
{
  .b.Config     = 0,
  .b.Signalling = 1,
  .b.Reserved1  = 0,
  .b.PinAssign  = 0,
  .b.Reserved2  = 0,
  .b.Reserved3  = 0,
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

USBPD_CertStatVdo_TypeDef CertStatVDO;
USBPD_ProductVdo_TypeDef ProductVDO;
uint16_t CurrentSVID;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  VDM Discovery identity callback (answer to Discover Identity message)
  * @param  Port current port number
  * @retval USBPD_DiscoveryIdentity_TypeDef
*/
static USBPD_StatusTypeDef USBPD_VDM_InformIdentity(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_DiscoveryIdentity_TypeDef *Identity)
{
  IDHeaderVDO.d32  = Identity->IDHeader.d32;
  CertStatVDO.d32  = Identity->CertStatVDO.d32;
  ProductVDO.d32   = Identity->ProductVDO.d32;
  
  SVIDInfo.NumSVIDs = 0;
  CurrentSVID       = 0;
  sDP_Status.b.ConnectStatus = 0x2;

  return USBPD_OK;
}

/**
  * @brief  Inform SVID callback
  * @param  Port current port number
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformSVID(uint8_t Port, USBPD_SOPType_TypeDef SOPType, uint16_t SVID)
{
  SVIDInfo.SVIDs[SVIDInfo.NumSVIDs++] = SVID;
  
  return USBPD_OK;
}

/**
  * @brief  Get SVID for Discovery Mode callback
  * @param  Port current port number
  * @retval USBPD status
  */
static uint16_t USBPD_VDM_GetSVID(uint8_t Port)
{
  if (CurrentSVID >= SVIDInfo.NumSVIDs)
    return 0;
  return SVIDInfo.SVIDs[CurrentSVID++];
}

/**
  * @brief  Get index of VDO for SVID (Enter mode) callback
  * @param  Port current port number
  * @param  SVID SVID used to be entered
  * @retval Index on the SVID VDO 
  */
static uint32_t USBPD_VDM_GetModeIndex(uint8_t Port, uint16_t SVID)
{
  uint32_t index = 0;
  for (index = 0; index < CurrentSVID; index++)
  {
    if (SVIDInfo.SVIDs[index] == SVID) break;
  }
  return (index + 1);
}

/**
  * @brief  Inform Mode callback
  * @param  Port current port number
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformMode(uint8_t Port, USBPD_SOPType_TypeDef SOPType, USBPD_ModeInfo_TypeDef *ModesInfo)
{
  sModesInfo.NumModes = ModesInfo->NumModes;
  for (uint32_t index = 0; index < sModesInfo.NumModes; index++)
  {
    sModesInfo.Modes[index] = ModesInfo->Modes[index];
  }
  
  sModesInfo.SVID = ModesInfo->SVID;
  sModesInfo.Nack = ModesInfo->Nack;
  
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
  uint32_t index = 0;
  if (ModeIndex != 0) index = ModeIndex - 1;
  sDP_Mode.d32 = sModesInfo.Modes[index];
  /* Send a DP status only if DFP capable */
  if ((sDP_Mode.b.SignalDirection == 1) || (sDP_Mode.b.SignalDirection == 3))
  {
    sDP_Status.b.ConnectStatus = 1;
    sDP_Status.b.Enable = 0;
    status = USBPD_OK;
  }
  
  return status;
}

/**
  * @brief  Inform Mode enter callback
  * @param  Port      current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformEnterMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  if (SVIDInfo.SVIDs[(ModeIndex - 1)] == SVID)
  {
    status = USBPD_OK;
    VDM_Mode_On = 1;
    sAttention.Command = SVDM_DP_STATUS;
    sAttention.SVID = SVID;
    sAttention.ModeIndex = ModeIndex;
    sAttention.VDO = 0;

    USBPD_PE_SVDM_RequestAttention(0);
  }

  return status;
}

/**
  * @brief  Inform Mode exit callback
  * @param  Port      current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be exited
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformExitMode(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;

  if (SVIDInfo.SVIDs[(ModeIndex - 1)] == SVID)
  {
    status = USBPD_OK;
    VDM_Mode_On = 2;
  }

  return status;
}

/**
  * @brief  VDM Attention callback
  * @param  Port            current port number
  * @param  pAttentionInfo  Pointer on USBPD_AttentionInfo_TypeDef
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_Attention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef **pAttentionInfo)
{
  *pAttentionInfo = &sAttention;
  if (sAttention.Command == SVDM_DP_STATUS)
   sAttention.VDO        = sDP_Status.d32;

  return USBPD_OK;
}

/**
  * @brief  VDM Inform Attention callback
  * @param  Port            current port number
  * @param  pAttentionInfo  Pointer on USBPD_AttentionInfo_TypeDef
  * @retval status
  */
static USBPD_StatusTypeDef USBPD_VDM_InformAttention(uint8_t PortNum, USBPD_AttentionInfo_TypeDef *pAttentionInfo)
{
  USBPD_DPStatus_TypeDef vdo_received;
  vdo_received.d32 = pAttentionInfo->VDO;
  
  if (vdo_received.b.ConnectStatus != 0)
  {
    if (vdo_received.b.ConnectStatus == sDP_Status.b.ConnectStatus)
    {
      sDP_Status.b.ConnectStatus = 2;
      sAttention.VDO        = sDP_Status.d32;
      sAttention.Command    = SVDM_DP_STATUS;
    }
    else
    {
      sDP_Config.b.Config   = 1;
      sAttention.VDO        = sDP_Config.d32;
      sAttention.Command    = SVDM_DP_CONFIG;
    }
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
  USBPD_VDM_Init(0, vdmCallbacks);
  return USBPD_OK;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

