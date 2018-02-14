/**
  ******************************************************************************
  * @file    usbd_bb.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-August-2016
  * @brief   Header file for the usbd_bb.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_BB_H
#define __USB_BB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_BB
  * @brief This file is the Header file for usbd_bb.c
  * @{
  */ 

/** @defgroup USBD_BB_Exported_Defines
  * @{
  */ 
#define USB_BB_CONFIG_DESC_SIZ        9
#define USB_BB_MAX_NUM_ALT_MODE       0x34
/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
  uint16_t            VendorID;
  uint8_t             bAlternateMode;
  uint8_t             iAlternateModeString;
}
USBD_BB_AlternateModeTypeDef;

typedef enum
{
  UNSPECIFIED_ERROR = 0,
  CONFIGURATION_NOT_ATTEMPTED,
  CONFIGURATION_UNSUCCESSFUL,
  CONFIGURATION_SUCCESSFUL,
}
BB_AlternateModeState;

/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_BB;
#define USBD_BB_CLASS    &USBD_BB
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 

#if (USBD_BOS_ENABLED == 1)
uint8_t USBD_BB_AddAlternateMode (uint8_t* hUSBDBOSDesc, USBD_BB_AlternateModeTypeDef* AlternateMode);
uint8_t USBD_BB_SetupPreferedAltMode (uint8_t* hUSBDBOSDesc, uint8_t PreferedAltModeIndex);
uint8_t USBD_BB_SetupAltModeState (uint8_t* hUSBDBOSDesc, uint8_t AltModeIndex, BB_AlternateModeState AltModeState);
#endif

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_BB_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
