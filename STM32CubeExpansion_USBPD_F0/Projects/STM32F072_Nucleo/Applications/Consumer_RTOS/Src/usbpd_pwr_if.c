/**
  ******************************************************************************
  * @file    usbpd_pwr_if.c
  * @author  System Lab
  * @version V1.0.0
  * @date    06-June-2016
  * @brief   This file contains power interface control functions.
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
#include "usbpd_pwr_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SOURCEPDO       1   /* Number of Surce PDO */
#define SINKPDO         0   /* Number of Sink PDO  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if (SOURCEPDO>0)
uint32_t PWR_SrcPDO[SOURCEPDO];
#endif
#if (SINKPDO>0)
uint32_t PWR_SinkPDO[SINKPDO];
#endif

/* Private function prototypes -----------------------------------------------*/
/* Functions to setup PDO */
uint32_t _PWR_FixexPDO(float _V_,float  _C_,USBPD_PWR_PeakCurrTypeDef _PK_, USBPD_PWR_ExtPowerTypeDef ExtPower);
uint32_t _PWR_VariablePDO(float  _MAXV_,float _MINV_,float _C_);
uint32_t _PWR_BatteryPDO(float _MAXV_,float _MINV_,float _PWR_);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the power supply board
  * @param  SrcPDO: Pointer to Power Data Objects provided by the Power Board
  * @param  nPDO: Number of supported Power Data Objects
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_Init(uint32_t* SrcPDO, uint8_t* nPDO) 
{
  uint8_t index = 0;
  
  /* Create the PDO. 
   *
   * USER should add here profiles provided by the
   * External Power Supply Board 
   * */
  PWR_SrcPDO[0] = _PWR_FixexPDO(5, 3, PeakEqual, ExtPowerOn);
  
  /* Copy the PDO to DPM */
  for(index=0;index<SOURCEPDO;index++)
  {
    SrcPDO[index]=PWR_SrcPDO[index];
  }
  
  /* Get the number of PDO */
  *nPDO = SOURCEPDO;

  return USBPD_OK; 
}

/**
  * @brief  Sets the required power profile, now it works only with Fixed ones  
  * @param  hport: The handle of the port
  * @param  profile: the number of the required Power Data Objects
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_SetProfile(uint8_t hport, uint8_t profile)
{
  USBPD_SNKFixedSupplyPDO_TypeDef fpdo;
  USBPD_PWR_PDOTypeDef  type;
  uint16_t voltage_target;
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* check for valid port */
  if ( !USBPD_PORT_IsValid(hport) )
  {
    return ret;
  }
  
  /* get the PDO type */
  fpdo.d32 = PWR_SrcPDO[profile];
  type = (USBPD_PWR_PDOTypeDef)(fpdo.d32 & 0x0003);
  
  /* Check if it is fixed type */
  if (type == Fixed)
  {
    /* Get the voltage in MV and set it */
    voltage_target = PWR_DECODE_mV(fpdo.b.VoltageIn50mVunits);
    HW_IF_PWR_SetVoltage(hport, voltage_target);
    ret = USBPD_OK;
  }
  return ret; 
}

/**
  * @brief  Resets the Power Board
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_PowerResetGlobal(void)
{ 
  /* Resets all the ports */
  for(int i = 0; i < USBPD_PORT_COUNT; i++)
  {
    USBPD_PWR_IF_PowerReset(i);
  }
  return USBPD_OK; 
}

/**
  * @brief  Resets the Power on a specified port
  * @param  hport: The handle of the port
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_PowerReset(uint8_t hport)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(hport))
  {
    return USBPD_ERROR;
  }
  /* Disable the output */
  USBPD_PWR_IF_Enable(hport, DISABLE);
  /* Put it to VSafe5V */
  USBPD_PWR_IF_SetProfile(hport,0);
  
  return USBPD_OK; 
}

/**
  * @brief  Checks if the power on a specified port is ready
  * @param  hport: The handle of the port
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t hport)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(hport))
  {
    return USBPD_ERROR;
  }
  
  return USBPD_OK; 
}

/**
  * @brief  Enables the power on a specified port
  * @param  hport: The handle of the port
  * @param  NewState: ENABLE (To enable power) or DISABLE (To disable the power)
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_Enable(uint8_t hport, FunctionalState NewState)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(hport))
  {
    return USBPD_ERROR;
  }
  /* Set the new state */
  HW_IF_PWR_Enable(hport, NewState);
  
  return  USBPD_OK;
}

/**
  * @brief  Checks if the power on a specified port is enabled
  * @param  hport: The handle of the port
  * @retval ENABLE or DISABLE 
*/
FunctionalState USBPD_PWR_IF_IsEnabled(uint8_t hport)
{ 
  /* Get the Status of the port */
  return USBPD_PORT_IsValid(hport) ? HW_IF_PWR_IsEnabled(hport) : DISABLE; 
}


/**
  * @brief  Reads the voltage and the current on a specified port
  * @param  hport: The handle of the port
  * @param  pVoltage: The Voltage in mV
  * @param  pCurrent: The Current in mA
  * @retval ENABLE or DISABLE 
*/
USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t hport, uint16_t *pVoltage, uint16_t *pCurrent) //mV, mA
{ 
  /* check for valid port */
  if (!USBPD_PORT_IsValid(hport))
  {
    return USBPD_ERROR;
  }
  
  /* USBPD_OK if at least one pointer is not null, otherwise USBPD_ERROR */
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* Get values from HW_IF */
  if (pVoltage != NULL)
  {
    *pVoltage = HW_IF_PWR_GetVoltage(hport);
    ret = USBPD_OK;
  }
  if (pCurrent != NULL)
  {
    *pCurrent = HW_IF_PWR_GetCurrent(hport);
    ret = USBPD_OK;
  }
  
  return ret;
}

/**
  * @brief  Create Fixes PDO object
  * @param  _V_: The handle of the port
  * @param  _C_: The voltage in V
  * @param  _PK_: The peak of current
  * @param  ExtPower: Source externally powered   
  * @retval The PDO object 
*/
uint32_t _PWR_FixexPDO(float _V_,float  _C_,USBPD_PWR_PeakCurrTypeDef _PK_, USBPD_PWR_ExtPowerTypeDef ExtPower)
{
  USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo;
  
  fixedpdo.d32 = 0;
  fixedpdo.b.MaxCurrentIn10mAunits = PWR_A(_C_);
  fixedpdo.b.VoltageIn50mVunits = PWR_V(_V_);
  fixedpdo.b.PeakCurrent = _PK_;
  fixedpdo.b.ExternallyPowered = ExtPower;
  fixedpdo.b.FixedSupply = Fixed;
  return fixedpdo.d32;
}

/**
  * @brief  Create Variable PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _C_: Max current in A
  * @retval The PDO object 
*/
uint32_t _PWR_VariablePDO(float  _MAXV_,float _MINV_,float _C_)
{
  USBPD_SRCVariableSupplyPDO_TypeDef variablepdo;
  
  variablepdo.d32 = 0;
  variablepdo.b.MaxCurrentIn10mAunits = PWR_A(_C_);
  variablepdo.b.MaxVoltageIn50mVunits = PWR_V(_MAXV_);
  variablepdo.b.MinVoltageIn50mVunits =  PWR_V(_MINV_);
  variablepdo.b.VariableSupply = Variable;
  return variablepdo.d32;
}

/**
  * @brief  Create Battery PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _PWR_: Max allowable power in W
  * @retval The PDO object 
*/
uint32_t _PWR_BatteryPDO(float _MAXV_,float _MINV_,float _PWR_)
{
  USBPD_SRCBatterySupplyPDO_TypeDef batterypdo;
  
  batterypdo.d32 = 0;
  batterypdo.b.MaxAllowablePowerIn250mWunits = PWR_W(_PWR_);
  batterypdo.b.MinVoltageIn50mVunits = PWR_V(_MINV_);
  batterypdo.b.MaxVoltageIn50mVunits = PWR_V(_MAXV_);
  batterypdo.b.Battery = Battery;
  return batterypdo.d32;  
}

/**
  * @brief  Enables the VConn on the port.
  * @param  port: the port, if available.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPD_PWR_IF_Enable_VConn(uint8_t hport, CCxPin_TypeDef cc)
{
  USBPDM1_Enable_VConn(hport, OTHER_CC(cc)); 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
