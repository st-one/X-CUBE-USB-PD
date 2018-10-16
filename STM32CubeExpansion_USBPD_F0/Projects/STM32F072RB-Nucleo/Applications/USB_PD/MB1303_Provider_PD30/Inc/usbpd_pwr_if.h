/**
  ******************************************************************************
  * @file    usbpd_pwr_if.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_pw_if.h.
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

#ifndef __USBPD_PW_IF_H_
#define __USBPD_PW_IF_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"

/** @addtogroup STM32_USBPD_USER
  * @{
  */

/** @addtogroup USBPD_USER
  * @{
  */

/** @addtogroup USBPD_USER_PWR_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define BSP_PWR_INVALID_VALUE           0xFFFFFFFFu   /* Invalid value set during issue with voltage setting */
#define BSP_PWR_TIMEOUT_PDO             250u          /* Timeout for PDO to PDO or PDO to APDO at 250ms */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup USBPD_USER_PWR_IF_Exported_Macros USBPD PWR IF Exported Macros
  * @{
  */

/* Macros used to convert values into PDO representation */
#define PWR_V_20MV(_V_)        ((uint16_t)(( (_V_) * 1000.0) / 20.0))   /* From Volt to 20mV multiples      */
#define PWR_V_50MV(_V_)        ((uint16_t)(( (_V_) * 1000.0) / 50.0))   /* From Volt to 50mV multiples      */
#define PWR_V_100MV(_V_)       ((uint16_t)(( (_V_) * 1000.0) / 100.0))  /* From Volt to 100mV multiples     */
#define PWR_A_10MA(_A_)        ((uint16_t)(( (_A_) * 1000.0) / 10.0))   /* From Ampere to 10mA multiples    */
#define PWR_A_50MA(_A_)        ((uint16_t)(( (_A_) * 1000.0) / 50.0))   /* From Ampere to 50mA multiples    */
#define PWR_W(_W_)             ((uint16_t)(( (_W_) * 1000.0) / 250.0))  /* From Watt to 250mW multiples     */

/* Macros used to get values from PDO representation */
#define PWR_DECODE_50MV(_Value_)           ((uint16_t)(( (float)(_Value_) * 50.0)))     /* From 50mV multiples to mV        */
#define PWR_DECODE_100MV(_Value_)          ((uint16_t)(( (float)(_Value_) * 100.0)))    /* From 100mV multiples to mV       */
#define PWR_DECODE_10MA(_Value_)           ((uint16_t)(( (float)(_Value_) * 10.0)))     /* From 10mA multiples to mA        */
#define PWR_DECODE_50MA(_Value_)           ((uint16_t)(( (float)(_Value_) * 50.0)))     /* From 50mA multiples to mA        */
#define PWR_DECODE_MW(_Value_)             ((uint16_t)(( (float)(_Value_) * 250.0)))    /* From 250mW multiples to mW       */

#define USBPD_PORT_IsValid(__Port__) ((__Port__) < (USBPD_PORT_COUNT))

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup USBPD_USER_PWR_IF_Exported_Functions USBPD PWR IF Exported Functions
  * @{
  */

/**
  * @brief  Initialize structures and variables related to power board profiles
  *         used by Sink and Source, for all available ports.
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Init(void);

/**
  * @brief  Sets the required power profile
  * @param  PortNum Port number
  * @param  Profile the number of the required Power Data Objects
  * @param  PreviousPowerProfile  Number of the previous required Power Data Objects
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SetProfile(uint8_t PortNum, uint8_t Profile, uint8_t PreviousPowerProfile);

/**
  * @brief  Resets the Power Board
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_PowerResetGlobal(void);

/**
  * @brief  Resets the Power on a specified port
  * @param  PortNum Port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_PowerReset(uint8_t PortNum);

/**
  * @brief  Checks if the power on a specified port is ready
  * @param  PortNum Port number
  * @param  Vsafe   Vsafe status based on @ref USBPD_VSAFE_StatusTypeDef
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe);

/**
  * @brief  Initialize the power on a specified port
  * @param  PortNum Port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_InitPower(uint8_t PortNum);

/**
  * @brief  Enable VBUS power on a specified port
  * @param  PortNum Port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_VBUSEnable(uint8_t PortNum);

/**
  * @brief  Disable VBUS power on a specified port
  * @param  PortNum Port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_VBUSDisable(uint8_t PortNum);

/**
  * @brief  Disable the SNK to stop current consumption 
  * @param  PortNum Port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SNKDisable(uint8_t PortNum);

/**
  * @brief  Checks if the power on a specified port is enabled
  * @param  PortNum Port number
  * @retval ENABLE or DISABLE
  */
USBPD_FunctionalState USBPD_PWR_IF_VBUSIsEnabled(uint8_t PortNum);

/**
  * @brief  Reads the voltage and the current on a specified port
  * @param  PortNum Port number
  * @param  pVoltage The Voltage in mV
  * @param  pCurrent The Current in mA
  * @retval ENABLE or DISABLE
  */
USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t PortNum, uint16_t *pVoltage, uint16_t *pCurrent);


/**
  * @brief  Allow PDO data reading from PWR_IF storage.
  * @param  PortNum Port number
  * @param  DataId Type of data to be read from PWR_IF
  *         This parameter can be one of the following values:
  *           @arg @ref USBPD_CORE_DATATYPE_SRC_PDO Source PDO reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_SNK_PDO Sink PDO reading requested
  * @param  Ptr Pointer on address where PDO values should be written (u32 pointer)
  * @param  Size Pointer on nb of u32 written by PWR_IF (nb of PDOs)
  * @retval None
  */
void USBPD_PWR_IF_GetPortPDOs(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t *Size);

#if defined(_SRC) || defined(_DRP)
/**
  * @brief  Find out SRC PDO pointed out by a position provided in a Request DO (from Sink).
  * @param  PortNum Port number
  * @param  RdoPosition RDO Position in list of provided PDO
  * @param  Pdo Pointer on PDO value pointed out by RDO position (u32 pointer)
  * @retval Status of search
  *         USBPD_OK : Src PDO found for requested DO position (output Pdo parameter is set)
  *         USBPD_FAIL : Position is not compliant with current Src PDO for this port (no corresponding PDO value)
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SearchRequestedPDO(uint8_t PortNum, uint32_t RdoPosition, uint32_t *Pdo);

#endif /* _SRC || _DRP */
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

#endif /* __USBPD_PW_IF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

