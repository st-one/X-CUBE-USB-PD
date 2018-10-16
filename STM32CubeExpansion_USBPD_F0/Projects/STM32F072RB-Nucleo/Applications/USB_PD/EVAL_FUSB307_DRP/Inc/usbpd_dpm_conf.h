/**
  ******************************************************************************
  * @file    usbpd_dpm_core.h
  * @author  MCD Application Team
  * @brief   Header file for usbpd_dpm_core.c file
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

#ifndef __USBPD_DPM_CONF_H_
#define __USBPD_DPM_CONF_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_pdo_defs.h"
#include "usbpd_dpm_user.h"

/* Define   ------------------------------------------------------------------*/
/* Define VID, PID,... manufacturer parameters */
#define USBPD_VID (0x0483u)     /*!< Vendor ID (assigned by the USB-IF)                     */
#define USBPD_PID (0x0002u)     /*!< Product ID (assigned by the manufacturer)              */
#define USBPD_XID (0xF0000003u) /*!< Value provided by the USB-IF assigned to the product   */

/* Exported typedef ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifndef __USBPD_DPM_CORE_C
extern const USBPD_SettingsTypeDef      DPM_Settings[USBPD_PORT_COUNT];
extern const USBPD_USER_SettingsTypeDef DPM_USER_Settings[USBPD_PORT_COUNT];
#else /* __USBPD_DPM_CORE_C */
const USBPD_SettingsTypeDef DPM_Settings[USBPD_PORT_COUNT] =
{
  {
    .PE_SupportedSOP = USBPD_SUPPORTED_SOP_SOP, /* Supported SOP : SOP      */
    .PE_SpecRevision = USBPD_SPECIFICATION_REV3,/* spec revision value                                     */
    .PE_DefaultRole = USBPD_PORTPOWERROLE_SNK,  /* Default port role                                       */
    .PE_RoleSwap = USBPD_TRUE,                  /* support port role swap                                  */
    .PE_VDMSupport = USBPD_FALSE,
    .PE_RespondsToDiscovSOP = USBPD_FALSE,      /*!< Can respond successfully to a Discover Identity */
    .PE_AttemptsDiscovSOP = USBPD_FALSE,        /*!< Can send a Discover Identity */
    .PE_PingSupport = USBPD_FALSE,              /* support ping                                            */
    .PE_CapscounterSupport = USBPD_FALSE,       /* support caps counter                                    */
    .CAD_RoleToggle = USBPD_TRUE,               /* cad role toogle                                         */
    .CAD_TryFeature = USBPD_FALSE,              /* cad try feature                                         */
    .CAD_AccesorySupport = USBPD_FALSE,         /* cas accessory support                                   */
    .PE_PD3_Support =                           /*!< PD3 SUPPORT FEATURE                                              */
    {
      .d.PE_UnchunkSupport                = USBPD_FALSE,  /*!< Unchunked mode Support                */
      .d.PE_FastRoleSwapSupport           = USBPD_FALSE,  /* support fast role swap only spec revsion 3.0            */
      .d.Is_GetPPSStatus_Supported        = USBPD_FALSE,  /*!< PPS message supported or not by DPM */
      .d.Is_SrcCapaExt_Supported          = USBPD_FALSE,  /*!< Source_Capabilities_Extended message supported or not by DPM */
      .d.Is_Alert_Supported               = USBPD_TRUE,   /*!< Alert message supported or not by DPM */
      .d.Is_GetStatus_Supported           = USBPD_TRUE,   /*!< Status message supported or not by DPM (Is_Alert_Supported should be enabled) */
      .d.Is_GetBattery_Supported          = USBPD_FALSE,  /*!< Battery_Status & Battery_Capa messages supported or not by DPM */
      .d.Is_GetManufacturerInfo_Supported = USBPD_TRUE,   /*!< Manufacturer_Info message supported or not by DPM */
      .d.Is_GetCountryCodes_Supported     = USBPD_FALSE,  /*!< Country_Codes message supported or not by DPM */
      .d.Is_GetCountryInfo_Supported      = USBPD_FALSE,  /*!< Country_Info message supported or not by DPM */
      .d.Is_SecurityRequest_Supported     = USBPD_FALSE,  /*!< Security_Response message supported or not by DPM */
      .d.Is_FirmUpdateRequest_Supported   = USBPD_FALSE,  /*!< Firmware update response message supported or not by DPM */
    },

    .CAD_SRCToogleTime          = 40,                    /* uint8_t CAD_SRCToogleTime; */
    .CAD_SNKToogleTime          = 40,                    /* uint8_t CAD_SNKToogleTime; */
  }
};


const USBPD_USER_SettingsTypeDef DPM_USER_Settings[USBPD_PORT_COUNT] =
{
  {
    .DPM_SNKRequestedPower =                                             /*!< Requested Power by the sink board                                    */
    {
      .MaxOperatingCurrentInmAunits = USBPD_CORE_PDO_SNK_FIXED_MAX_CURRENT,
      .OperatingVoltageInmVunits    = USBPD_BOARD_REQUESTED_VOLTAGE_MV,
      .MaxOperatingVoltageInmVunits = USBPD_BOARD_MAX_VOLTAGE_MV,
      .MinOperatingVoltageInmVunits = USBPD_BOARD_MIN_VOLTAGE_MV,
      .OperatingPowerInmWunits      = (USBPD_CORE_PDO_SNK_FIXED_MAX_CURRENT * USBPD_BOARD_REQUESTED_VOLTAGE_MV)/1000,
      .MaxOperatingPowerInmWunits   = (USBPD_CORE_PDO_SNK_FIXED_MAX_CURRENT * USBPD_BOARD_MAX_VOLTAGE_MV)/1000
    },
    .PE_DataSwap = USBPD_TRUE,                  /* support data swap    */
    .PE_VconnSwap = USBPD_FALSE,                /* support VCONN swap   */
    .DPM_ManuInfoPort =                      /*!< Manufacturer information used for the port            */
    {
      .VID = USBPD_VID,                      /*!< Vendor ID (assigned by the USB-IF)        */
      .PID = USBPD_PID,                      /*!< Product ID (assigned by the manufacturer) */
      .ManuString = "STMicroelectronics",    /*!< Vendor defined byte array                 */
    },
  },
};
#endif /* !__USBPD_DPM_CORE_C */

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DPM_CONF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
