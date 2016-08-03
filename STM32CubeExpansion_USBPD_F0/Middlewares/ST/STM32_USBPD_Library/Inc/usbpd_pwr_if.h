/**
  ******************************************************************************
  * @file    usbpd_pwr_if.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   This file contains the headers of usbpd_pw_if.h.
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

#ifndef __USBPD_PW_IF_H_
#define __USBPD_PW_IF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_conf.h"
#include "usbpd_def.h"
#include "usbpd_hw_if.h"

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_PWR_PDOTypeDef definition
  * @brief  PDO type structure definition
  * @{
  */
typedef enum {
  Fixed = 0x00,         /*!< Fixed Supply PDO Type                            */
  Battery = 0x01,       /*!< Battery Supply PDO Type                          */
  Variable = 0x02,      /*!< Variable Supply (non-battery) PDO Type           */
} USBPD_PWR_PDOTypeDef; 
/** 
  * @}
  */

/** @defgroup USBPD_PWR_PeakCurrTypeDef definition
  * @brief  Fixed Power Source Peak Current Capability type structure definition
  * @{
  */
typedef enum
{
  PeakEqual = 0x00,     /*!< Peak current equals */
  PeakOver1 = 0x01,     /*!< Overload Capabilities:
  1. Peak current equals 150% IOC for 1ms @ 5% duty cycle (low current equals 97% IOC for 19ms)
  2. Peak current equals 125% IOC for 2ms @ 10% duty cycle (low current equals 97% IOC for 18ms)
  3. Peak current equals 110% IOC for 10ms @ 50% duty cycle (low current equals 90% IOC for 10ms*/
  PeakOver2 = 0x02,     /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 150% IOC for 2ms @ 10% duty cycle (low current equals 94% IOC for 18ms)
  3. Peak current equals 125% IOC for 10ms @ 50% duty cycle (low current equals 75% IOC for 10ms)  */
  PeakOver3 = 0x03,     /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 175% IOC for 2ms @ 10% duty cycle (low current equals 92% IOC for 18ms)
  3. Peak current equals 150% IOC for 10ms @ 50% duty cycle (low current equals 50% IOC for 10ms)*/
}USBPD_PWR_PeakCurrTypeDef;
/** 
  * @}
  */

/** @defgroup USBPD_PWR_ExtPowerTypeDef definition
  * @brief  Fixed Power Source Externally Powered type structure definition
  * @{
  */
typedef enum
{
  ExtPowerOff   = 0,    /*!< Source self powered */
  ExtPowerOn    = 1     /*!< Source externally powered */
}USBPD_PWR_ExtPowerTypeDef;
/** 
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Macros used to convert values into PDO representation */
#define PWR_V(_V_)             ((uint16_t)(( (_V_) * 1000.0) / 50.0))   /* From Volt to 50mV multiples      */
#define PWR_A(_A_)             ((uint16_t)(( (_A_) * 1000.0) / 10.0))   /* From Ampere to 10mA multiples    */
#define PWR_W(_W_)             ((uint16_t)(( (_W_) * 1000.0) / 250.0))  /* From Watt to 250mW multiples     */

/* Macros used to get values from PDO representation */
#define PWR_DECODE_mV(_Value_)             ((uint16_t)(( (float)(_Value_) * 50.0)))     /* From 50mV multiples to  mV       */
#define PWR_DECODE_mA(_Value_)             ((uint16_t)(( (float)(_Value_) * 10.0)))     /* From 10mA multiples to mA        */
#define PWR_DECODE_mW(_Value_)             ((uint16_t)(( (float)(_Value_) * 250.0)))    /* From 250mW multiples to mW       */    

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the power supply board
  * @param  SrcPDO: Pointer to Power Data Objects provided by the Power Board
  * @param  nPDO: Number of supported Power Data Objects
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Init(uint32_t* SrcPDO, uint8_t* nPDO);

/**
  * @brief  Sets the required power profile  
  * @param  hport: The handle of the port
  * @param  profile: the number of the required Power Data Objects
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SetProfile(uint8_t hport, uint8_t profile); 

/**
  * @brief  Resets the Power Board
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_PowerResetGlobal(void);

/**
  * @brief  Resets the Power on a specified port
  * @param  hport: The handle of the port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_PowerReset(uint8_t hport);

/**
  * @brief  Checks if the power on a specified port is ready
  * @param  hport: The handle of the port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t hport);

/**
  * @brief  Enables the power on a specified port
  * @param  hport: The handle of the port
  * @param  NewState: ENABLE (To enable power) or DISABLE (To disable the power)
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Enable(uint8_t hport, FunctionalState NewState);

/**
  * @brief  Checks if the power on a specified port is enabled
  * @param  hport: The handle of the port
  * @retval ENABLE or DISABLE 
  */
FunctionalState USBPD_PWR_IF_IsEnabled(uint8_t hport);

/**
  * @brief  Reads the voltage and the current on a specified port
  * @param  hport: The handle of the port
  * @param  pVoltage: The Voltage in mV
  * @param  pCurrent: The Current in mA
  * @retval ENABLE or DISABLE 
  */
USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t hport, uint16_t *pVoltage, uint16_t *pCurrent);

/**
  * @brief  Enables the VConn on the port.
  * @param  hport: the port handle.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPD_PWR_IF_Enable_VConn(uint8_t hport, CCxPin_TypeDef cc);

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PW_IF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
