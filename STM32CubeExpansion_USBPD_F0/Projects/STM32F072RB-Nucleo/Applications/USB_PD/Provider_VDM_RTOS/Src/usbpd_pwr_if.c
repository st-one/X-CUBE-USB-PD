/**
  ******************************************************************************
  * @file    usbpd_pwr_if.c
  * @author  System Lab
  * @version V1.2.0
  * @date    17-Jan-2017
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
#include "usbpd_hw_if.h"

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief  USBPD Port PDO Storage Structure definition
  */
typedef struct
{
  USBPD_PortPDO_TypeDef    SourcePDO;        /*!< SRC Power Data Objects */
  USBPD_PortPDO_TypeDef    SinkPDO;          /*!< SNK Power Data Objects */

}USBPD_PWR_Port_PDO_Storage_TypeDef;

/* Private define ------------------------------------------------------------*/
#define PORT0_NB_SOURCEPDO       1   /* Number of Source PDOs (applicable for port 0) */
#define PORT0_NB_SINKPDO         0   /* Number of Sink PDOs (applicable for port 0)   */

#if (PORT0_NB_SOURCEPDO > USBPD_MAX_NB_PDO)
#error "Nb of Source PDO is exceeding stack capabilities"
#endif
#if (PORT0_NB_SINKPDO > USBPD_MAX_NB_PDO)
#error "Nb of Sink PDO is exceeding stack capabilities"
#endif

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
  * @brief  USBPD Port PDO Storage array declaration
  */
USBPD_PWR_Port_PDO_Storage_TypeDef PWR_Port_PDO_Storage[USBPD_PORT_COUNT];

/* Definition of Source PDO for Port 0 */
#if (PORT0_NB_SOURCEPDO > 0)
uint32_t PORT0_PDO_ListSRC[PORT0_NB_SOURCEPDO];
#endif
/* Definition of Sink PDO for Port 0 */
#if (PORT0_NB_SINKPDO > 0)
uint32_t PORT0_PDO_ListSNK[PORT0_NB_SINKPDO];
#endif

/* Kind of temporary handle */
USBPD_PortPowerRole_TypeDef PWR_IF_Role[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
/* Functions to initialize Source PDOs */
uint32_t _PWR_SRCFixedPDO(float  _C_, float _V_,
                          USBPD_CORE_PDO_PeakCurr_TypeDef _PK_, 
                          USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                          USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                          USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                          USBPD_CORE_PDO_USBSuspendSupport_TypeDef UsbSuspendSupport,
                          USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport);
uint32_t _PWR_SRCVariablePDO(float _MAXV_, float _MINV_, float _C_);
uint32_t _PWR_SRCBatteryPDO(float _MAXV_,float _MINV_,float _PWR_);
/* Functions to initialize Sink PDOs */
uint32_t _PWR_SNKFixedPDO(float  _C_, float _V_, 
                          USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                          USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                          USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                          USBPD_CORE_PDO_HigherCapability_TypeDef HigherCapab,
                          USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport);
uint32_t _PWR_SNKVariablePDO(float  _MAXV_,float _MINV_,float _C_);
uint32_t _PWR_SNKBatteryPDO(float _MAXV_,float _MINV_,float _PWR_);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize structures and variables related to power board profiles
  *         used by Sink and Source, for all available ports.
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_Init(void) 
{
  
  /* Initialize PDOs :
   * USER should here defines SRC and SNK PDO values for all available ports.
   * PDO values are stored in PWR_IF local variables (as PORT0_PDO_ListSRC, PORT0_PDO_ListSNK, ...).
   *
   * PWR_IF functions exist for building PDO, for either SRC or SNK, 
   * and for Fixed, Variable or Battery PDO types.
   *
   * Alternatively, a PDO content value could be built as constant values at compilation time, 
   * instead being computed at execution time. Building/accessing u32 values is 
   * possible by using corresponding defines for mask, shift and values in usbpd_def.h 
   *
   * As an example, below code is initializing 1 SRC PDO for Port 0.
   * This code is equivalent to :
   *    PORT0_PDO_ListSRC[0] =
   *        ( ((PWR_A(3)) << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)                                   |
   *          ((PWR_V(5)) << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)                                       |
   *          (USBPD_CORE_PDO_PEAKEQUAL << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos)                     |
   *          (USBPD_CORE_PDO_DRD_NOT_SUPPORTED << USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Pos)             |
   *          (USBPD_CORE_PDO_USBCOMM_NOT_CAPABLE << USBPD_PDO_SRC_FIXED_USBCOMM_Pos)               |
   *          (USBPD_CORE_PDO_NOT_EXT_POWERED << USBPD_PDO_SRC_FIXED_EXT_POWER_Pos)                 |
   *          (USBPD_CORE_PDO_USBSUSP_NOT_SUPPORTED << USBPD_PDO_SRC_FIXED_USBSUSPEND_Pos)          |
   *          (USBPD_CORE_PDO_DRP_NOT_SUPPORTED << USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Pos)             |
   *          USBPD_PDO_TYPE_FIXED
   *        );
   *    PORT0_PDO_ListSRC[0] =
   *        ( ((PWR_A(3)) << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)                                   |
   *          ((PWR_V(5)) << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)                                       |
   *          USBPD_PDO_SRC_FIXED_PEAKCURRENT_EQUAL                                                 |
   *          USBPD_PDO_SRC_FIXED_DRD_NOT_SUPPORTED                                                 |
   *          USBPD_PDO_SRC_FIXED_USBCOMM_NOT_SUPPORTED                                             |
   *          USBPD_PDO_SRC_FIXED_EXT_POWER_NOT_AVAILABLE                                           |
   *          USBPD_PDO_SRC_FIXED_USBSUSPEND_NOT_SUPPORTED                                          |
   *          USBPD_PDO_SRC_FIXED_DRP_NOT_SUPPORTED                                                 |
   *          USBPD_PDO_TYPE_FIXED
   *        );
   */
  PORT0_PDO_ListSRC[0] = _PWR_SRCFixedPDO(3, 5,
                                          USBPD_CORE_PDO_PEAKEQUAL,
                                          USBPD_CORE_PDO_DRD_NOT_SUPPORTED,
                                          USBPD_CORE_PDO_USBCOMM_NOT_CAPABLE,
                                          USBPD_CORE_PDO_NOT_EXT_POWERED,
                                          USBPD_CORE_PDO_USBSUSP_NOT_SUPPORTED,
                                          USBPD_CORE_PDO_DRP_NOT_SUPPORTED);

  /* Set links to PDO values and number for Port 0 in PWR_IF array.
   *
   */
  PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.ListOfPDO = PORT0_PDO_ListSRC ;
  PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.NumberOfPDO = PORT0_NB_SOURCEPDO;

  PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.ListOfPDO = NULL ;
  PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.NumberOfPDO = PORT0_NB_SINKPDO;

  return USBPD_OK; 
}

/**
  * @brief  Set power role
  * @param  PortNum Port number
  * @param  role  Power role
  * @retval USBPD status
*/
void USBPD_PWR_IF_SetRole(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role)
{
  PWR_IF_Role[PortNum] = role;
}

/**
  * @brief  Sets the required power profile, now it works only with Fixed ones  
  * @param  PortNum Port number
  * @param  profile the number of the required Power Data Objects
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_SetProfile(uint8_t PortNum, uint8_t profile)
{
  USBPD_SNKFixedSupplyPDO_TypeDef fpdo;
  USBPD_CORE_PDO_Type_TypeDef     type;
  uint16_t                        voltage_target;
  USBPD_StatusTypeDef             ret = USBPD_ERROR;
  
  /* Check if valid port */
  if ( !USBPD_PORT_IsValid(PortNum) )
  {
    return ret;
  }

  /* Check if profile nb is valid for this port */
  if (profile >= PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO)
  {
    return ret;
  }

  /* Extract type of selected PDO */
  fpdo.d32 = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO[profile];
  type = (USBPD_CORE_PDO_Type_TypeDef)((fpdo.d32 & USBPD_PDO_TYPE_Msk) >> USBPD_PDO_TYPE_Pos);
  
  /* Case PDO is a fixed type */
  if (type == USBPD_CORE_PDO_TYPE_FIXED)
  {
    /* Get the voltage in MV and set it */
    voltage_target = PWR_DECODE_mV(fpdo.b.VoltageIn50mVunits);
    HW_IF_PWR_SetVoltage(PortNum, voltage_target);
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
  * @param  PortNum Port number
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_PowerReset(uint8_t PortNum)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(PortNum))
  {
    return USBPD_ERROR;
  }
  /* Disable the output */
  USBPD_PWR_IF_Enable(PortNum, DISABLE, PWR_IF_Role[PortNum]);
  /* Put it to VSafe5V */
  USBPD_PWR_IF_SetProfile(PortNum,0);
  
  return USBPD_OK; 
}

/**
  * @brief  Checks if the power on a specified port is ready
  * @param  PortNum Port number
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t PortNum)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(PortNum))
  {
    return USBPD_ERROR;
  }
  
  return USBPD_OK; 
}

/**
  * @brief  Enables the power on a specified port
  * @param  PortNum Port number
  * @param  NewState ENABLE (To enable power) or DISABLE (To disable the power)
  * @param  Role     Power role
  * @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_PWR_IF_Enable(uint8_t PortNum, FunctionalState NewState, USBPD_PortPowerRole_TypeDef Role)
{
  /* check for valid port */
  if (!USBPD_PORT_IsValid(PortNum))
  {
    return USBPD_ERROR;
  }
  /* Set the new state */
  HW_IF_PWR_Enable(PortNum, NewState, Role);
  
  return  USBPD_OK;
}

/**
  * @brief  Checks if the power on a specified port is enabled
  * @param  PortNum Port number
  * @retval ENABLE or DISABLE 
*/
FunctionalState USBPD_PWR_IF_IsEnabled(uint8_t PortNum)
{ 
  /* Get the Status of the port */
  return USBPD_PORT_IsValid(PortNum) ? HW_IF_PWR_IsEnabled(PortNum) : DISABLE; 
}


/**
  * @brief  Reads the voltage and the current on a specified port
  * @param  PortNum Port number
  * @param  pVoltage: The Voltage in mV
  * @param  pCurrent: The Current in mA
  * @retval USBPD_ERROR or USBPD_OK 
*/
USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t PortNum, uint16_t *pVoltage, uint16_t *pCurrent) //mV, mA
{ 
  /* check for valid port */
  if (!USBPD_PORT_IsValid(PortNum))
  {
    return USBPD_ERROR;
  }
  
  /* USBPD_OK if at least one pointer is not null, otherwise USBPD_ERROR */
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* Get values from HW_IF */
  if (pVoltage != NULL)
  {
    *pVoltage = HW_IF_PWR_GetVoltage(PortNum);
    ret = USBPD_OK;
  }
  if (pCurrent != NULL)
  {
    *pCurrent = HW_IF_PWR_GetCurrent(PortNum);
    ret = USBPD_OK;
  }
  
  return ret;
}

/**
  * @brief  Create SRC Fixed PDO object
  * @param  _C_: Current in A
  * @param  _V_: voltage in V
  * @param  _PK_: The peak of current
  * @param  DRDSupport: Data Role Swap support indication
  * @param  UsbCommCapable: USB communications capability indication
  * @param  ExtPower: Port externally powered indication
  * @param  UsbSuspendSupport: USB Suspend support indication
  * @param  DRPSupport: Dual Role Power support indication
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SRCFixedPDO(float  _C_, float _V_,
                          USBPD_CORE_PDO_PeakCurr_TypeDef _PK_, 
                          USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                          USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                          USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                          USBPD_CORE_PDO_USBSuspendSupport_TypeDef UsbSuspendSupport,
                          USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport)
{
  USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo;
  
  fixedpdo.d32 = 0;
  fixedpdo.b.MaxCurrentIn10mAunits       = PWR_A(_C_);
  fixedpdo.b.VoltageIn50mVunits          = PWR_V(_V_);
  fixedpdo.b.PeakCurrent                 = _PK_;
  fixedpdo.b.DataRoleSwap                = DRDSupport;
  fixedpdo.b.USBCommunicationsCapable    = UsbCommCapable;
  fixedpdo.b.ExternallyPowered           = ExtPower;
  fixedpdo.b.USBSuspendSupported         = UsbSuspendSupport;
  fixedpdo.b.DualRolePower               = DRPSupport;
  fixedpdo.b.FixedSupply                 = USBPD_CORE_PDO_TYPE_FIXED;
  return fixedpdo.d32;
}

/**
  * @brief  Create SRC Variable PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _C_: Max current in A
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SRCVariablePDO(float _MAXV_, float _MINV_, float _C_)
{
  USBPD_SRCVariableSupplyPDO_TypeDef variablepdo;
  
  variablepdo.d32 = 0;
  variablepdo.b.MaxCurrentIn10mAunits = PWR_A(_C_);
  variablepdo.b.MaxVoltageIn50mVunits = PWR_V(_MAXV_);
  variablepdo.b.MinVoltageIn50mVunits = PWR_V(_MINV_);
  variablepdo.b.VariableSupply        = USBPD_CORE_PDO_TYPE_VARIABLE;
  return variablepdo.d32;
}

/**
  * @brief  Create SRC Battery PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _PWR_: Max allowable power in W
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SRCBatteryPDO(float _MAXV_,float _MINV_,float _PWR_)
{
  USBPD_SRCBatterySupplyPDO_TypeDef batterypdo;
  
  batterypdo.d32 = 0;
  batterypdo.b.MaxAllowablePowerIn250mWunits = PWR_W(_PWR_);
  batterypdo.b.MinVoltageIn50mVunits         = PWR_V(_MINV_);
  batterypdo.b.MaxVoltageIn50mVunits         = PWR_V(_MAXV_);
  batterypdo.b.Battery                       = USBPD_CORE_PDO_TYPE_BATTERY;
  return batterypdo.d32;  
}

/**
  * @brief  Create SNK Fixed PDO object
  * @param  _C_: Current in A
  * @param  _V_: voltage in V
  * @param  DRDSupport: Data Role Swap support indication
  * @param  UsbCommCapable: USB communications capability indication
  * @param  ExtPower: Port externally powered indication
  * @param  HigherCapab: Sink needs more than vSafe5V to provide full functionality indication
  * @param  DRPSupport: Dual Role Power support indication
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SNKFixedPDO(float  _C_, float _V_, 
                          USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                          USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                          USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                          USBPD_CORE_PDO_HigherCapability_TypeDef HigherCapab,
                          USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport)
{
  USBPD_SNKFixedSupplyPDO_TypeDef fixedpdo;
  
  fixedpdo.d32 = 0;
  fixedpdo.b.OperationalCurrentIn10mAunits = PWR_A(_C_);
  fixedpdo.b.VoltageIn50mVunits            = PWR_V(_V_);
  fixedpdo.b.DataRoleSwap                  = DRDSupport;
  fixedpdo.b.USBCommunicationsCapable      = UsbCommCapable;
  fixedpdo.b.ExternallyPowered             = ExtPower;
  fixedpdo.b.HigherCapability              = HigherCapab;
  fixedpdo.b.DualRolePower                 = DRPSupport;
  fixedpdo.b.FixedSupply                   = USBPD_CORE_PDO_TYPE_FIXED;
      
  return fixedpdo.d32;
}

/**
  * @brief  Create SNK Variable PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _C_: Max current in A
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SNKVariablePDO(float  _MAXV_,float _MINV_,float _C_)
{
  USBPD_SRCVariableSupplyPDO_TypeDef variablepdo;
  
  variablepdo.d32 = 0;
  variablepdo.b.MaxCurrentIn10mAunits = PWR_A(_C_);
  variablepdo.b.MaxVoltageIn50mVunits = PWR_V(_MAXV_);
  variablepdo.b.MinVoltageIn50mVunits =  PWR_V(_MINV_);
  variablepdo.b.VariableSupply        = USBPD_CORE_PDO_TYPE_VARIABLE;
  return variablepdo.d32;
}

/**
  * @brief  Create SNK Battery PDO object
  * @param  _MAXV_: Max voltage in V
  * @param  _MINV_: Min voltage in V
  * @param  _PWR_: Max allowable power in W
  * @retval PDO object value (expressed on u32)
*/
uint32_t _PWR_SNKBatteryPDO(float _MAXV_,float _MINV_,float _PWR_)
{
  USBPD_SRCBatterySupplyPDO_TypeDef batterypdo;
  
  batterypdo.d32 = 0;
  batterypdo.b.MaxAllowablePowerIn250mWunits = PWR_W(_PWR_);
  batterypdo.b.MinVoltageIn50mVunits         = PWR_V(_MINV_);
  batterypdo.b.MaxVoltageIn50mVunits         = PWR_V(_MAXV_);
  batterypdo.b.Battery                       = USBPD_CORE_PDO_TYPE_BATTERY;
  return batterypdo.d32;  
}

/**
  * @brief  Enables the VConn on the port.
  * @param  port: the port, if available.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPD_PWR_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef cc)
{
  USBPD_HW_IF_Enable_VConn(PortNum, OTHER_CC(cc)); 
}

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
void USBPD_PWR_IF_GetPortPDOs(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t *Size)
{
  uint32_t   nbpdo, index;
  uint32_t   *ptpdoarray;
  
  /* Check if valid port */
  if (USBPD_PORT_IsValid(PortNum))
  {
    /* According to type of PDO to be read, set pointer on values and nb of elements */
    if (DataId == USBPD_CORE_DATATYPE_SRC_PDO)
    {
      nbpdo = PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO;
      ptpdoarray = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO;
    }
    else if (DataId == USBPD_CORE_DATATYPE_SNK_PDO)
    {
      nbpdo = PWR_Port_PDO_Storage[PortNum].SinkPDO.NumberOfPDO;
      ptpdoarray = PWR_Port_PDO_Storage[PortNum].SinkPDO.ListOfPDO;
    }    
    else
    {
      nbpdo = 0;
    }    

    /* Copy PDO data in output buffer */
    for (index = 0; index < nbpdo; index++)
    {
      USPBPD_WRITE32((uint32_t *)(Ptr + index), *ptpdoarray);
      ptpdoarray++;
    }
    /* Set nb of read PDO (nb of u32 elements); */
    *Size = nbpdo;
  }
}

/**
  * @brief  Find out SRC PDO pointed out by a position provided in a Request DO (from Sink).
  * @param  PortNum Port number
  * @param  RdoPosition RDO Position in list of provided PDO
  * @param  Pdo Pointer on PDO value pointed out by RDO position (u32 pointer)
  * @retval Status of search
  *         USBPD_OK : Src PDO found for requested DO position (output Pdo parameter is set)
  *         USBPD_FAIL : Position is not compliant with current Src PDO for this port (no corresponding PDO value)
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SearchRequestedPDO(uint8_t PortNum, uint32_t RdoPosition, uint32_t *Pdo)
{
  if((RdoPosition == 0) || (RdoPosition > PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO))
  {
    /* Invalid PDO index */
    USBPD_ErrLog("USBPD_PWR_IF_SearchRequestedPDO: Invalid PDOs index\r");
    return USBPD_FAIL;
  }

  *Pdo = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO[RdoPosition - 1];
  return USBPD_OK;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
