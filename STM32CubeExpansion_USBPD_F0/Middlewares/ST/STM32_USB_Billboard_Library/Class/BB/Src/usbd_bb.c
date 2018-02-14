/**
  ******************************************************************************
  * @file    usbd_bb.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-August-2016
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB BillBoard Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as BillBoard Device
  *           - Error management
 * @verbatim
  *      
  *          ===================================================================      
  *                                BillBoard Class Description
  *          =================================================================== 
  *           This module manages the BillBoard class V1.0.0 following the "Device Class Definition
  *           for BillBoard Devices (BB) Version 1.0a Apr 15, 2015".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as an USB BillBoard device
  *             - Enumeration & management of BillBoard device supported alternate modes
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_bb.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_BB 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_BB_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_BB_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_BB_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_BB_Private_FunctionPrototypes
  * @{
  */

static uint8_t  USBD_BB_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_BB_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_BB_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_BB_DataIn (USBD_HandleTypeDef *pdev, 
                                uint8_t epnum);

static uint8_t  USBD_BB_DataOut (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_BB_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_BB_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_BB_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  *USBD_BB_GetOtherSpeedCfgDesc (uint16_t *length);

/**
  * @}
  */ 

/** @defgroup USBD_BB_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_BB = 
{
  USBD_BB_Init, /*Init*/
  USBD_BB_DeInit, /*DeInit*/
  USBD_BB_Setup, /*Setup*/
  NULL, /*EP0_TxSent*/  
  USBD_BB_EP0_RxReady, /*EP0_RxReady*/
  USBD_BB_DataIn, /*DataIn*/
  USBD_BB_DataOut, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_BB_GetCfgDesc,
  USBD_BB_GetCfgDesc,
  USBD_BB_GetOtherSpeedCfgDesc,
  USBD_BB_GetDeviceQualifierDesc,
};

/* USB Standard Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_BB_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,     /*bLength*/
  USB_DESC_TYPE_DEVICE_QUALIFIER, /*bDescriptorType*/
  0x00,                           /*bcdUSB*/
  0x03,
  0x11,                           /*bDeviceClass*/
  0x00,                           /*bDeviceSubClass*/
  0x00,                           /*bDeviceProtocol*/
  0x40,                           /*bMaxPacketSize0*/
  0x01,                           /*bNumConfigurations*/
  0x00,                           /*bReserved*/
};

/* USB device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_BB_CfgDesc[USB_BB_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_BB_CONFIG_DESC_SIZ,     /* wTotalLength: Bytes returned */
  0x00,
  0x00,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0x80,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
} ;

/* USB device Other Speed Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_BB_OtherSpeedCfgDesc[USB_BB_CONFIG_DESC_SIZ]   __ALIGN_END  =
{
  0x09,   /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,   
  USB_BB_CONFIG_DESC_SIZ,
  0x00,
  0x00,   /* bNumInterfaces: 0 interface */
  0x01,   /* bConfigurationValue: */
  0x04,   /* iConfiguration: */
  0xE0,   /* bmAttributes: */
  0x32,   /* MaxPower 100 mA */
} ;

/**
  * @}
  */ 

/** @defgroup USBD_BB_Private_Functions
  * @{
  */

/**
  * @brief  USBD_BB_Init
  *         Initialize the BB interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_BB_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(cfgidx);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_Init
  *         DeInitialize the BB layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_BB_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(cfgidx);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_Setup
  *         Handle the BB specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_BB_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(req);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_BB_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_BB_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_BB_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  
  return USBD_OK;
}

/**
  * @brief  USBD_BB_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_BB_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_BB_CfgDesc);
  return USBD_BB_CfgDesc;
}

/**
  * @brief  USBD_BB_GetOtherSpeedCfgDesc 
  *         return other speed configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t  *USBD_BB_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_BB_OtherSpeedCfgDesc);
  return USBD_BB_OtherSpeedCfgDesc;
}

/**
  * @brief  DeviceQualifierDescriptor 
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_BB_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_BB_DeviceQualifierDesc);
  return USBD_BB_DeviceQualifierDesc;
}

#if (USBD_BOS_ENABLED == 1)
/**
  * @brief  USBD_BB_AddAlternateMode 
  *         Add a supported alternate mode
  * @param  hUSBDBOSDesc: Device BOS Descriptor
  * @param  AlternateMode: Alternate Mode Definition
  * @retval status
  */
uint8_t USBD_BB_AddAlternateMode (uint8_t* hUSBDBOSDesc, USBD_BB_AlternateModeTypeDef* AlternateMode)
{
  if (hUSBDBOSDesc[29] == USB_BB_MAX_NUM_ALT_MODE)
    return USBD_FAIL;

  /* AlternateMode VendorID */
  hUSBDBOSDesc[hUSBDBOSDesc[2]]   = (uint8_t)(AlternateMode->VendorID & 0x00FF);
  hUSBDBOSDesc[hUSBDBOSDesc[2]+1] = (uint8_t)((AlternateMode->VendorID & 0xFF00) >> 8);
  /* AlternateMode Index */
  hUSBDBOSDesc[hUSBDBOSDesc[2]+2] = AlternateMode->bAlternateMode;
  /* AlternateMode String Descriptor */
  hUSBDBOSDesc[hUSBDBOSDesc[2]+3] = AlternateMode->iAlternateModeString;

  /* BOS Length */
  hUSBDBOSDesc[2]  += 4;
  /* BillBoard Capability Length */
  hUSBDBOSDesc[25] += 4;
  /*Number of supported Alternate Modes*/
  hUSBDBOSDesc[29] += 1;

  return USBD_OK;
}

/**
  * @brief  USBD_BB_SetupPreferedAltMode
  *         Setup index of the prefered alternate mode
  * @param  hUSBDBOSDesc: Device BOS Descriptor
  * @param  PreferedAltModeIndex: Index of prefered alternate mode
  * @retval status
  */
uint8_t USBD_BB_SetupPreferedAltMode (uint8_t* hUSBDBOSDesc, uint8_t PreferedAltModeIndex)
{
  if (PreferedAltModeIndex >= hUSBDBOSDesc[29])
    return USBD_FAIL;

  /*Index of prefered supported alternate modes*/
  hUSBDBOSDesc[30] = PreferedAltModeIndex;

  return USBD_OK;
}

/**
  * @brief  USBD_BB_SetupAltModeState
  *         Setup index of the prefered alternate mode
  * @param  hUSBDBOSDesc: Device BOS Descriptor
  * @param  AltModeIndex: Index of an alternate mode
  * @param  AltModeState: Alternate mode state
  * @retval status
  */
uint8_t USBD_BB_SetupAltModeState (uint8_t* hUSBDBOSDesc, uint8_t AltModeIndex, BB_AlternateModeState AltModeState)
{
  uint8_t BOSAltModeConfigStateIndex;

  if (AltModeIndex >= hUSBDBOSDesc[29])
    return USBD_FAIL;

  /* Compute the alternate mode index in the BOS descriptor */
  BOSAltModeConfigStateIndex = 33 + (AltModeIndex / 4);

  /* Clear the old state for the alternate mode */
  hUSBDBOSDesc[BOSAltModeConfigStateIndex] &= ~(0x03 << (((AltModeIndex % 4)) * 2));

  /* Setup the alternate mode state */
  hUSBDBOSDesc[BOSAltModeConfigStateIndex] |= ((AltModeState & 0x03) << (((AltModeIndex % 4)) * 2));

  return USBD_OK;
}
#endif

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
