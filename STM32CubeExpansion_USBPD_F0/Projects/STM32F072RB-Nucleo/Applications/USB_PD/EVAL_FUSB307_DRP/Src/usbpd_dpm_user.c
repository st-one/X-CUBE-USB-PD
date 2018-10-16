/**
  ******************************************************************************
  * @file    usbpd_dpm_user.c
  * @author  MCD Application Team
  * @brief   USBPD DPM user code
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

#define USBPD_DPM_USER_C
/* Includes ------------------------------------------------------------------*/
#include "usbpd_tcpci.h"
#include "fusb305.h"
#include "usbpd_core.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"
#include "usbpd_dpm_user.h"
#if  defined(_TRACE)
#include "usbpd_trace.h"
#endif /* _GUI_INTERFACE || _TRACE */
#include "usbpd_pwr_if.h"
#include "string.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_USER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_DEFINES USBPD USER Private Defines
  * @{
  */

#define DPM_TIMER_ENABLE_MSK      ((uint16_t)0x8000U)       /*!< Enable Timer Mask                                                        */
#define DPM_TIMER_READ_MSK        ((uint16_t)0x7FFFU)       /*!< Read Timer Mask                                                          */

#define DPM_BOX_MESSAGES_MAX      30u

/* Timer used to check if need to send an alert */
#define DPM_TIMER_ALERT             100u  /*!< 100ms */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_MACROS USBPD USER Private Macros
  * @{
  */
#define DPM_START_TIMER(_PORT_, _TIMER_, _TIMEOUT_) DPM_Ports[_PORT_]._TIMER_ = (_TIMEOUT_) |  DPM_TIMER_ENABLE_MSK; \
                                                    osMessagePut(DPMMsgBox, DPM_USER_EVENT_TIMER, osWaitForever);
#define IS_DPM_TIMER_RUNNING(_PORT_, _TIMER_)       ((DPM_Ports[_PORT_]._TIMER_ & DPM_TIMER_READ_MSK) > 0)
#define IS_DPM_TIMER_EXPIRED(_PORT_, _TIMER_)       (DPM_TIMER_ENABLE_MSK == DPM_Ports[_PORT_]._TIMER_)

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_VARIABLES USBPD USER Private Variables
  * @{
  */
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];

extern osMessageQId  AlarmMsgBox;


/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_FUNCTIONS USBPD USER Private Functions
  * @{
  */
static  void DPM_SNK_GetSelectedPDO(uint8_t PortNum, uint8_t IndexSrcPDO, uint16_t RequestedVoltage, USBPD_SNKRDO_TypeDef* Rdo, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject);
static int32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_SNKPowerRequest_TypeDef* PtrRequestedPower);
static USBPD_StatusTypeDef DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static USBPD_StatusTypeDef DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static void DPM_AssertRd(uint8_t PortNum);
static void DPM_AssertRp(uint8_t PortNum);
static uint32_t CheckDPMTimers(void);


/**
  * @}
  */

/* Exported functions ------- ------------------------------------------------*/
/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS USBPD USER Exported Functions
  * @{
  */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 USBPD USER Exported Functions called by DPM CORE
  * @{
  */

/**
  * @brief  Initialize DPM (port power role, PWR_IF, CAD and PE Init procedures)
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_UserInit(void)
{

  /* PWR SET UP */
  USBPD_PWR_IF_Init();

  osMessageQDef(MsgBox, DPM_BOX_MESSAGES_MAX, uint32_t);
  DPMMsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);
  osThreadDef(DPM, USBPD_DPM_UserExecute, osPriorityLow, 0, 120);

  if(NULL == osThreadCreate(osThread(DPM), &DPMMsgBox))
  {
    return USBPD_ERROR;
  }

  return USBPD_OK;
}

/**
  * @brief  User delay implementation which is OS dependant
  * @param  Time time in ms
  * @retval None
  */
void USBPD_DPM_WaitForTime(uint32_t Time)
{
  osDelay(Time);
}

/**
  * @brief  User processing time, it is recommended to avoid blocking task for long time
  * @param  None
  * @retval None
  */
void USBPD_DPM_UserExecute(void const *argument)
{
  /* User code implementation */
  uint32_t _timing = osWaitForever;
  osMessageQId  queue = *(osMessageQId *)argument;

  do{
    osEvent event = osMessageGet(queue, _timing);
    switch (((DPM_USER_EVENT)event.value.v & 0xF))
    {
    case DPM_USER_EVENT_TIMER:

      /* check if Alert timer is expired */
      if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_0].DPM_TimerAlert)
      {
        /* Restart alert timer */
        DPM_START_TIMER(USBPD_PORT_0, DPM_TimerAlert, DPM_TIMER_ALERT);
        DPM_Ports[USBPD_PORT_0].DPM_MeasuredCurrent = HW_IF_PWR_GetCurrent(USBPD_PORT_0);
        if (DPM_Ports[USBPD_PORT_0].DPM_MeasuredCurrent > 3600)
        {
          USBPD_DPM_RequestHardReset(USBPD_PORT_0);
        }
        else
        {
          if (DPM_Ports[USBPD_PORT_0].DPM_MeasuredCurrent > 3400)
          {
            if (0 == (DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OCP))
            {
              USBPD_ADO_TypeDef alert = {0};
              alert.b.TypeAlert = USBPD_ADO_TYPE_ALERT_OCP;
              USBPD_DPM_RequestAlert(USBPD_PORT_0, alert);
              DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert |= alert.b.TypeAlert;
            }
          }
          else
          {
            /* Reset of the OCP bit */
            DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OCP;
            if (DPM_Ports[USBPD_PORT_0].DPM_MeasuredCurrent > DPM_Ports[USBPD_PORT_0].DPM_RequestedCurrent)
            {
              if (0 == (DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OPERATING_COND))
              {
                USBPD_ADO_TypeDef alert = {0};
                alert.b.TypeAlert = USBPD_ADO_TYPE_ALERT_OPERATING_COND;
                USBPD_DPM_RequestAlert(USBPD_PORT_0, alert);
                DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert |= alert.b.TypeAlert;
              }
            }
            else
            {
              /* Reset alert bits */
              DPM_Ports[USBPD_PORT_0].DPM_SendAlert.b.TypeAlert = 0;
            }
          }
        }
      }

      break;
    default:
      break;
    }
    _timing = CheckDPMTimers();
  }
  while(1);
}

/**
  * @brief  function used to manage user timer.
  * @param  PortNum Port number
  * @retval None
  */
void USBPD_DPM_UserTimerCounter(uint8_t PortNum)
{
  if((DPM_Ports[PortNum].DPM_TimerAlert & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerAlert--;
  }
}

/**
  * @brief  UserCableDetection reporting events on a specified port from CAD layer.
  * @param  PortNum The handle of the port
  * @param  State CAD state
  * @retval None
  */
void USBPD_DPM_UserCableDetection(uint8_t PortNum, USBPD_CAD_EVENT State)
{
  switch(State)
  {
  case USBPD_CAD_EVENT_ATTEMC:

    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
    }

    DPM_Ports[PortNum].DPM_IsConnected = 1;
    break;

  case USBPD_CAD_EVENT_ATTACHED:
    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
    }

    DPM_Ports[PortNum].DPM_IsConnected = 1;
    break;

  case USBPD_CAD_EVENT_LEGACY:
    break;
  case USBPD_CAD_EVENT_DETACHED :
  case USBPD_CAD_EVENT_EMC :
  default :
    /* reset all values received from port partner */
    memset(&DPM_Ports[PortNum], 0, sizeof(DPM_Ports[PortNum]));

    break;
  }
}

/**
  * @}
  */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP2 USBPD USER Exported Callbacks functions called by PE
  * @{
  */

/**
  * @brief  Callback function called by PE layer when HardReset message received from PRL
  * @param  PortNum     The current port number
  * @param  CurrentRole the current role
  * @param  Status      status on hard reset event
  * @retval None
  */
void USBPD_DPM_HardReset(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status)
{
  /* Stop Alert timer */
  DPM_Ports[PortNum].DPM_TimerAlert    = 0;
  DPM_Ports[PortNum].DPM_SendAlert.d32 = 0;
  switch (Status)
  {
  case USBPD_HR_STATUS_START_ACK:
  case USBPD_HR_STATUS_START_REQ:
    if (USBPD_PORTPOWERROLE_SRC == CurrentRole)
    {
      /* Restore default Role in case of Power Swap failing due to no PS_READY from Sink (TC PC.E2)  */
      DPM_AssertRp(PortNum);
      /* Reset the power supply */
      DPM_TurnOffPower(PortNum, USBPD_PORTPOWERROLE_SRC);
    }
    else
    {
      USBPD_PWR_IF_VBUSIsEnabled(PortNum);
    }
    break;
  case USBPD_HR_STATUS_COMPLETED:
    if (USBPD_PORTPOWERROLE_SRC == CurrentRole)
    {
      /* Reset the power supply */
      DPM_TurnOnPower(PortNum,CurrentRole);
    }
    break;
  default:
      break;
  }
}

/**
  * @brief  Request the DPM to setup the new power level.
  * @param  PortNum The current port number
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_DPM_SetupNewPower(uint8_t PortNum)
{
  USBPD_StatusTypeDef status;
  uint8_t rdoposition, previous_rdoposition;

  /* Retrieve Request DO position from DPM handle : RDO position in the table of PDO (possible value from 1 to 7) */
  rdoposition = DPM_Ports[PortNum].DPM_RDOPosition;
  previous_rdoposition = DPM_Ports[PortNum].DPM_RDOPositionPrevious;

  /* Check if get the right pdo position */
  if (rdoposition > 0)
  {
    status = USBPD_PWR_IF_SetProfile(PortNum, rdoposition-1, previous_rdoposition);
  }
  else
  {
    /* Put it to VSafe5V */
    status = USBPD_PWR_IF_SetProfile(PortNum, 0, 0);
  }

  return status;
}

/**
  * @brief  Evaluate the swap request from PE.
  * @param  PortNum The current port number
  * @retval USBPD_ACCEPT, USBPD_WAIT, USBPD_REJECT
  */
USBPD_StatusTypeDef USBPD_DPM_EvaluatePowerRoleSwap(uint8_t PortNum)
{
  return USBPD_ACCEPT;
}

/**
  * @brief  Callback function called by PE to inform DPM about PE event.
  * @param  PortNum The current port number
  * @param  EventType @ref USBPD_NotifyEvent_TypeDef
  * @param  EventVal @ref USBPD_NotifyEventValue_TypeDef
  * @retval None
  */
void USBPD_DPM_Notification(uint8_t PortNum, USBPD_NotifyEventValue_TypeDef EventVal)
{
  switch(EventVal)
  {
    /***************************************************************************
                              Power Notification
    */
    case USBPD_NOTIFY_POWER_EXPLICIT_CONTRACT :
      /* Power ready means an explicit contract has been establish and Power is available */
      if ((USBPD_SPECIFICATION_REV3 == DPM_Params[PortNum].PE_SpecRevision)
       && (USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole))
      {
        DPM_START_TIMER(PortNum, DPM_TimerAlert, DPM_TIMER_ALERT);
      }
      break;
    /*
                              End Power Notification
     ***************************************************************************/
    /***************************************************************************
                               REQUEST ANSWER NOTIFICATION
    */
    case USBPD_NOTIFY_REQUEST_ACCEPTED:
      /* Update DPM_RDOPosition only if current role is SNK */
      if (USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
      {
        USBPD_SNKRDO_TypeDef rdo;
        rdo.d32                             = DPM_Ports[PortNum].DPM_RequestDOMsg;
        DPM_Ports[PortNum].DPM_RDOPosition  = rdo.GenericRDO.ObjectPosition;
      }
    break;
    /*
                              End REQUEST ANSWER NOTIFICATION
     ***************************************************************************/
    case USBPD_NOTIFY_STATE_SNK_READY:
      {
      }
      break;

    case USBPD_NOTIFY_STATE_SRC_DISABLED:
      {
        /* SINK Port Partner is not PD capable. Legacy cable may have been connected
           In this state, VBUS is set to 5V */
      }
      break;
    default :
      break;
  }
}

/**
  * @brief  Request DPM to confirm if current contract is still valid.
  * @param  PortNum Port number
  * @retval USBPD_OK, USBPD_ERROR
*/
USBPD_StatusTypeDef USBPD_DPM_IsContractStillValid(uint8_t PortNum)
{
  return USBPD_OK;
}

/**
  * @brief  DPM callback to allow PE to retrieve information from DPM/PWR_IF.
  * @param  PortNum Port number
  * @param  DataId  Type of data to be updated in DPM based on @ref USBPD_CORE_DataInfoType_TypeDef
  * @param  Ptr     Pointer on address where DPM data should be written (u32 pointer)
  * @param  Size    Pointer on nb of u8 written by DPM
  * @retval None
  */
void USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t *Size)
{
  uint32_t index = 0;

  /* Check type of information targeted by request */
  switch (DataId)
  {
    /* Case Port Source PDO Data information :
    Case Port SINK PDO Data information :
    Call PWR_IF PDO reading request.
    */
  case USBPD_CORE_DATATYPE_SRC_PDO :
  case USBPD_CORE_DATATYPE_SNK_PDO :
    USBPD_PWR_IF_GetPortPDOs(PortNum, DataId, Ptr, Size);
    *Size *= 4;
    break;

    /* Case Port Received Source PDO Data information (from distant port) */
  case USBPD_CORE_DATATYPE_RCV_SRC_PDO :
    for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO; index++)
    {
      *(uint32_t*)(Ptr + index) = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index];
    }
    *Size = (DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO * 4);
    break;

    /* Case Port Received Sink PDO Data information (from distant port) */
  case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
    for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO; index++)
    {
      *(uint32_t*)(Ptr + index) = DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index];
    }
    *Size = (DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO * 4);
    break;

    /* Case Requested voltage value Data information */
  case USBPD_CORE_DATATYPE_REQ_VOLTAGE :
    *Ptr = DPM_Ports[PortNum].DPM_RequestedVoltage;
    *Size = 4;
    break;

    /* Case Request message DO (from Sink to Source) Data information */
  case USBPD_CORE_DATATYPE_REQUEST_DO :
    *Ptr = DPM_Ports[PortNum].DPM_RequestDOMsg;
    *Size = 4;
    break;


  case USBPD_CORE_INFO_STATUS :
    {
      USBPD_SDB_TypeDef  infostatus = {
          .InternalTemp = 0,          /*!< Source or Sink internal temperature in degrees centigrade */
          .PresentInput = 0,          /*!< Present Input                                             */
          .PresentBatteryInput = 0,   /*!< Present Battery Input                                     */
          .EventFlags = 0,            /*!< Event Flags                                               */
          .TemperatureStatus = 0,     /*!< Temperature                                               */
      };

      *Size = sizeof(USBPD_SDB_TypeDef);
      memcpy((uint8_t *)Ptr, &infostatus, *Size);
    }
    break;

  case USBPD_CORE_MANUFACTURER_INFO :
    {
      USBPD_MIDB_TypeDef* manu_info;
      manu_info = (USBPD_MIDB_TypeDef*)&DPM_USER_Settings[PortNum].DPM_ManuInfoPort;

      /* Manufacturer Info Target must be a range 0..1 */
      /* Manufacturer Info Ref must be a range 0..7    */
      if((DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget > USBPD_MANUFINFO_TARGET_BATTERY)
      || (DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoRef > USBPD_MANUFINFO_REF_MAX_VALUES))
      {
        /* No manufacturer info to transmit */
        *Size = 4; /* VID (2) + .PID(2) */
      }
      else
      {
        if (USBPD_MANUFINFO_TARGET_PORT_CABLE_PLUG == DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget)
        {
          /* Manufacturer info requested for the port */
          /* VID(2) + .PID(2) + .ManuString("STMicroelectronics") */
          *Size = 4 + strlen((char*)(DPM_USER_Settings[PortNum].DPM_ManuInfoPort.ManuString));
        }
        else
        {
          /* Manufacturer info requested for the battery (not available yet) */
          /* No manufacturer info to transmit */
          *Size = 4; /* VID (2) + .PID(2) */
        }
      }

      /* Copy Manufacturer Info into data area for transmission */
      memcpy((uint8_t*)Ptr, (uint8_t *)manu_info, *Size);
    }
    break;

  case USBPD_CORE_ALERT:
    {
      /*!< Over-Current Protection event when set (Source only, for Sink Reserved and Shall be set to zero) */
      /* Bit reserved for a Sink */
      if ((USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
               && (USBPD_ADO_TYPE_ALERT_OCP == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OCP)))
      {
        /* Reset OCP bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OCP;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Over-Temperature Protection event when set. */
      else if (USBPD_ADO_TYPE_ALERT_OTP == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OTP))
      {
        /* Reset OTP bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OTP;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Operating Condition Change when set */
      else if (USBPD_ADO_TYPE_ALERT_OPERATING_COND == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OPERATING_COND))
      {
        /* Reset OP COND bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OPERATING_COND;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Source Input Change Event when set */
      else if (USBPD_ADO_TYPE_ALERT_SRC_INPUT == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_SRC_INPUT))
      {
        /* Reset SRC input bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_SRC_INPUT;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Over-Voltage Protection event when set (Sink only, for Source Reserved and Shall be set to zero) */
      /* Bit reserved for a Source */
      else
      {
        if ((USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
               && (USBPD_ADO_TYPE_ALERT_OVP == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OVP)))
        {
          /* Reset OVP bit */
          DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OVP;
          /* Send a Get_Status */
          goto _ctrl_msg;
        }
      }

_ctrl_msg:
      /* Post GET_STATUS message */
      USBPD_DPM_RequestGetStatus(PortNum);
    }
    break;
  default :
    *Size = 0;
    break;
  }
}

/**
  * @brief  DPM callback to allow PE to update information in DPM/PWR_IF.
  * @param  PortNum Port number
  * @param  DataId  Type of data to be updated in DPM based on @ref USBPD_CORE_DataInfoType_TypeDef
  * @param  Size    Nb of bytes to be updated in DPM
  * @retval None
  */
void USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t Size)
{
  uint32_t index;

  /* Check type of information targeted by request */
  switch (DataId)
  {
    /* Case requested DO position Data information :
    */
  case USBPD_CORE_DATATYPE_RDO_POSITION :
    if (Size == 4)
    {
      DPM_Ports[PortNum].DPM_RDOPosition = *Ptr;
      DPM_Ports[PortNum].DPM_RDOPositionPrevious = *Ptr;
    }
    break;

    /* Case requested Voltage Data information :
    */
  case USBPD_CORE_DATATYPE_REQ_VOLTAGE :
    if (Size == 4)
    {
      DPM_Ports[PortNum].DPM_RequestedVoltage = *Ptr;
    }
    break;

    /* Case Received Source PDO values Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_SRC_PDO :
    if (Size <= (USBPD_MAX_NB_PDO * 4))
    {
      DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO = (Size / 4);
      /* Copy PDO data in DPM Handle field */
      for (index = 0; index < (Size / 4); index++)
      {
        DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index] = LE32(Ptr + index);
      }
    }
    break;

    /* Case Received Sink PDO values Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
    if (Size <= (USBPD_MAX_NB_PDO * 4))
    {
      DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO = (Size / 4);
      /* Copy PDO data in DPM Handle field */
      for (index = 0; index < (Size / 4); index++)
      {
        DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index] = LE32(Ptr + index);
      }
    }
    break;

    /* Case Received Request PDO Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_REQ_PDO :
    if (Size == 4)
    {
      DPM_Ports[PortNum].DPM_RcvRequestDOMsg = *Ptr;
    }
    break;

    /* Case Request message DO (from Sink to Source) Data information :
    */
  case USBPD_CORE_DATATYPE_REQUEST_DO :
    if (Size == 4)
    {
      DPM_Ports[PortNum].DPM_RcvRequestDOMsg = *Ptr;
    }
    break;

  case USBPD_CORE_INFO_STATUS :
    {
      uint8_t* info_status;
      info_status = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvStatus;
      memcpy(info_status, Ptr, Size);
    }
    break;
  case USBPD_CORE_GET_MANUFACTURER_INFO:
    {
      uint8_t* temp = (uint8_t*)Ptr;
      DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget = *temp;
      DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoRef    = *(temp + 1);
    }
    break;
    case USBPD_CORE_ALERT:
      {
        uint8_t*  alert;
        alert = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvAlert.d32;
        memcpy(alert, Ptr, Size);
      }
      break;


    /* In case of unexpected data type (Set request could not be fulfilled) :
    */
  default :
    break;
  }
}

/**
  * @brief  Evaluate received Request Message from Sink port
  * @param  pdhandle Pointer to USB PD handle
  * @param  PtrPowerObject  Pointer on the power data object
  * @retval USBPD status : USBPD_ACCEPT, USBPD_REJECT, USBPD_WAIT, USBPD_GOTOMIN
  */
USBPD_StatusTypeDef USBPD_DPM_EvaluateRequest(uint8_t PortNum, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject)
{
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_PDO_TypeDef pdo;
  uint32_t pdomaxcurrent = 0;
  uint32_t rdomaxcurrent = 0, rdoopcurrent = 0, rdoobjposition = 0;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];

  rdo.d32 = pdhandle->DPM_RcvRequestDOMsg;
  rdoobjposition  = rdo.GenericRDO.ObjectPosition;
  pdhandle->DPM_RDOPosition = 0;

  /* Check if RDP can be met within the supported PDOs by the Source port */
  /* USBPD_DPM_EvaluateRequest: Evaluate Sink Request\r */
  /* USBPD_DPM_EvaluateRequest: Check if RDP can be met within the supported PDOs by the Source port\r */

  /* Search PDO in Port Source PDO list, that corresponds to Position provided in Request RDO */
  if (USBPD_PWR_IF_SearchRequestedPDO(PortNum, rdoobjposition, &pdo.d32) != USBPD_OK)
  {
    /* Invalid PDO index */
    /* USBPD_DPM_EvaluateRequest: Invalid PDOs index */
    return USBPD_REJECT;
  }

  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      pdomaxcurrent = pdo.SRCFixedPDO.MaxCurrentIn10mAunits;
      rdomaxcurrent = rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits;
      rdoopcurrent  = rdo.FixedVariableRDO.OperatingCurrentIn10mAunits;
      DPM_Ports[PortNum].DPM_RequestedCurrent = rdoopcurrent * 10;
      if(rdoopcurrent > pdomaxcurrent)
      {
        /* Sink requests too much operating current */
        /* USBPD_DPM_EvaluateRequest: Sink requests too much operating current*/
        return USBPD_REJECT;
      }

      if(rdomaxcurrent > pdomaxcurrent)
      {
        /* Sink requests too much maximum operating current */
        /* USBPD_DPM_EvaluateRequest: Sink requests too much maximum operating current */
        return USBPD_REJECT;
      }
    }
    break;
  case USBPD_CORE_PDO_TYPE_BATTERY:
  case USBPD_CORE_PDO_TYPE_VARIABLE:
  default:
    {
      return USBPD_REJECT;
    }
  }

  /* Set RDO position and requested voltage in DPM port structure */
  pdhandle->DPM_RequestedVoltage = pdo.SRCFixedPDO.VoltageIn50mVunits * 50;
  pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
  pdhandle->DPM_RDOPosition = rdoobjposition;

  /* Save the power object */
  *PtrPowerObject = pdo.GenericPDO.PowerObject;

  /* Accept the requested power */
  /* USBPD_DPM_EvaluateRequest: Sink requested %d mV %d mA for operating current from %d to %d mA\r",
               pdo.SRCFixedPDO.VoltageIn50mVunits * 50, pdo.SRCFixedPDO.MaxCurrentIn10mAunits * 10,
               rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits * 10, rdo.FixedVariableRDO.OperatingCurrentIn10mAunits * 10 */
  /* USBPD_DPM_EvaluateRequest: Source accepts the requested power */
  return USBPD_ACCEPT;
}

/**
  * @brief  Evaluate received Capabilities Message from Source port and prepare the request message
  * @param  PortNum         Port number
  * @param  PtrRequestData  Pointer on selected request data object
  * @param  PtrPowerObject  Pointer on the power data object
  * @retval None
  */
void USBPD_DPM_SNK_EvaluateCapabilities(uint8_t PortNum, uint32_t *PtrRequestData, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject)
{
  uint32_t mv = 0, mw = 0, ma = 0;
  USBPD_PDO_TypeDef  pdo;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];
  int32_t pdoindex = 0;

  pdhandle->DPM_RequestedVoltage = 0;

  /* USBPD_DPM_EvaluateCapabilities: Port Partner Requests Max Voltage */

  /* Find the Pdo index for the requested voltage */
  pdoindex = DPM_FindVoltageIndex(PortNum, &(puser->DPM_SNKRequestedPower));

  /* Initialize RDO */
  rdo.d32 = 0;

  /* If could not find desired pdo index, then return error */
  if (pdoindex == -1)
  {
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "PE_EvaluateCapability: could not find desired voltage", sizeof("PE_EvaluateCapability: could not find desired voltage"));
#endif /* _TRACE */
    rdo.FixedVariableRDO.ObjectPosition = 1;
    rdo.FixedVariableRDO.OperatingCurrentIn10mAunits =  puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
    rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
    rdo.FixedVariableRDO.CapabilityMismatch = 1;
    DPM_Ports[PortNum].DPM_RequestedCurrent = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits;
    /* USBPD_DPM_EvaluateCapabilities: Mismatch, could not find desired pdo index */

    pdhandle->DPM_RequestDOMsg = rdo.d32;
    *PtrRequestData = rdo.d32;
    return;
  }

  /* Extract power information from Power Data Object */
  pdo.d32 = pdhandle->DPM_ListOfRcvSRCPDO[pdoindex];

  /* Set the Object position */
  USBPD_PDO_TypeDef     snk_fixed_pdo;
  uint32_t size = 0;

  USBPD_PWR_IF_GetPortPDOs(PortNum, USBPD_CORE_DATATYPE_SNK_PDO, &snk_fixed_pdo.d32, &size);
  rdo.GenericRDO.ObjectPosition               = pdoindex + 1;
  rdo.GenericRDO.NoUSBSuspend                 = 1;
  rdo.GenericRDO.USBCommunicationsCapable     = snk_fixed_pdo.SNKFixedPDO.USBCommunicationsCapable;

  *PtrPowerObject = pdo.GenericPDO.PowerObject;

  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo = pdo.SRCFixedPDO;
      mv = PWR_DECODE_50MV(fixedpdo.VoltageIn50mVunits);
      ma = PWR_DECODE_10MA(fixedpdo.MaxCurrentIn10mAunits);
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      mw = ma * mv; /* mW */
      DPM_Ports[PortNum].DPM_RequestedCurrent = ma;
      rdo.FixedVariableRDO.OperatingCurrentIn10mAunits  = ma / 10;
      rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = ma / 10;
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
        rdo.FixedVariableRDO.CapabilityMismatch = 1;
        /* USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power */
      }
    }
    break;
  case USBPD_CORE_PDO_TYPE_BATTERY:
    {
      USBPD_SRCBatterySupplyPDO_TypeDef batterypdo = pdo.SRCBatteryPDO;
      mv = PWR_DECODE_50MV(batterypdo.MinVoltageIn50mVunits);
      mw = PWR_DECODE_MW(batterypdo.MaxAllowablePowerIn250mWunits);
      mw = USBPD_MIN(mw, puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits); /* mW */
      ma = mw / mv; /* mA */
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      DPM_Ports[PortNum].DPM_RequestedCurrent       = ma;
      mw = ma * mv; /* mW */
      rdo.BatteryRDO.ObjectPosition                 = pdoindex + 1;
      rdo.BatteryRDO.OperatingPowerIn250mWunits     = mw / 250;
      rdo.BatteryRDO.MaxOperatingPowerIn250mWunits  = mw / 250;
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        rdo.BatteryRDO.CapabilityMismatch = 1;
        /* Mismatch, less power offered than the operating power */
      }
      /* USBPD_DPM_EvaluateCapabilities: Battery Request Data Object %u: %d mW", rdo.d32, mw */
    }
    break;
  case USBPD_CORE_PDO_TYPE_VARIABLE:
    {
      //USBPD_SRCVariableSupplyPDO_TypeDef variablepdo = pdo.SRCVariablePDO;
    }
    break;
  default:
    break;
  }

  pdhandle->DPM_RequestDOMsg = rdo.d32;

  *PtrRequestData = pdhandle->DPM_RequestDOMsg;
  /* Get the requested voltage */
  pdhandle->DPM_RequestedVoltage = mv;
}

/**
  * @brief  Power role swap status update
  * @param  PortNum Port number
  * @param  CurrentRole the current role
  * @param  Status status on power role swap event
  */
void USBPD_DPM_PowerRoleSwap(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_PRS_Status_TypeDef Status)
{
    switch (Status)
    {
    case USBPD_PRS_STATUS_VBUS_OFF:
      if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
      {
        
        /* In case of power role swap keep VCONN On */
        DPM_TurnOffPower(PortNum, CurrentRole);
      }
      break;
    case USBPD_PRS_STATUS_SRC_RP2RD:
      DPM_AssertRd(PortNum);
      break;
    case USBPD_PRS_STATUS_SNK_RD2RP:
      DPM_AssertRp(PortNum);
      break;
    case USBPD_PRS_STATUS_VBUS_ON:
      DPM_TurnOnPower(PortNum, CurrentRole);

      /* Check if still connected */
      osMessagePut(AlarmMsgBox, (PortNum << 8 | 1), osWaitForever);
      break;
    default:
      break;
    }
}


/**
  * @brief  DPM callback to allow PE to forward extended message information.
  * @param  PortNum Port number
  * @param  MsgType Type of message to be handled in DPM
  *         This parameter can be one of the following values:
  *           @arg @ref USBPD_EXT_SECURITY_REQUEST Security Request extended message
  *           @arg @ref USBPD_EXT_SECURITY_RESPONSE Security Response extended message
  * @param  Ptr Pointer on address Extended Message data could be read (u8 pointer)
  * @param  Size Nb of u8 that compose Extended message
  * @retval None
  */
void USBPD_DPM_ExtendedMessageReceived(uint8_t PortNum, USBPD_ExtendedMsg_TypeDef MsgType, uint8_t *ptrData, uint16_t DataSize)
{
  if (DataSize == 0)
  {
    /* No data received. */
    return;
  }
  
  switch(USBPD_EXTENDED_MESSAGE | MsgType)
  {
#ifdef _FWUPD
    case USBPD_EXT_FIRM_UPDATE_REQUEST :
      uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
      USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, USBPD_EXT_FIRM_UPDATE_RESPONSE, data, 4);
    case USBPD_EXT_FIRM_UPDATE_RESPONSE :
      break;
#endif /* _FWUPD */
    default:
      break;
  }
}


/**
  * @brief  DPM callback used to know user choice about Data Role Swap.
  * @param  PortNum Port number
  * @retval USBPD_REJECT, UBPD_ACCEPT
  */
USBPD_StatusTypeDef USBPD_DPM_EvaluateDataRoleSwap(uint8_t PortNum)
{
  USBPD_StatusTypeDef status = USBPD_REJECT;
  if (USBPD_TRUE == DPM_USER_Settings[PortNum].PE_DataSwap)
  {
    status = USBPD_ACCEPT;
  }
  return status;
}

/**
  * @brief  Callback to be used by PE to check is VBUS is ready or present
  * @param  PortNum Port number
  * @param  Vsafe   Vsafe status based on @ref USBPD_VSAFE_StatusTypeDef
  * @retval USBPD_DISABLE or USBPD_ENABLE
  */
USBPD_FunctionalState USBPD_DPM_IsPowerReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe)
{
  return ((USBPD_OK == USBPD_PWR_IF_SupplyReady(PortNum, Vsafe)) ? USBPD_ENABLE : USBPD_DISABLE);
}
/**
  * @}
  */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP3 USBPD USER Functions PD messages requests
  * @{
  */

/**
  * @brief  Request the PE to send a hard reset
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestHardReset(uint8_t PortNum)
{
  return USBPD_PE_Request_HardReset(PortNum);
}

/**
  * @brief  Request the PE to send a cable reset.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestCableReset(uint8_t PortNum)
{
  /* Not yet implemeneted. */
  return USBPD_ERROR;
}

/**
  * @brief  Request the PE to send a GOTOMIN message
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGotoMin(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GOTOMIN, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to send a request message.
  * @param  PortNum     The current port number
  * @param  IndexSrcPDO Index on the selected SRC PDO (value between 1 to 7)
  * @param  RequestedVoltage Requested voltage (in MV and use mainly for APDO)
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestMessageRequest(uint8_t PortNum, uint8_t IndexSrcPDO, uint16_t RequestedVoltage)
{
  USBPD_StatusTypeDef status = USBPD_ERROR;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_CORE_PDO_Type_TypeDef pdo_object;

  rdo.d32 = 0;

  DPM_SNK_GetSelectedPDO(PortNum, (IndexSrcPDO - 1), RequestedVoltage, &rdo, &pdo_object);

  status = USBPD_PE_Send_Request(PortNum, rdo.d32, pdo_object);

  return status;
}

/**
  * @brief  Request the PE to send a GET_SRC_CAPA message
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapability(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SRC_CAP, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to send a GET_SNK_CAPA message
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapability(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SNK_CAP, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to perform a Data Role Swap.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestDataRoleSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_DR_SWAP, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to perform a Power Role Swap.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_PR_SWAP, USBPD_SOPTYPE_SOP);

}

/**
  * @brief  Request the PE to perform a VCONN Swap.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestVconnSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_VCONN_SWAP, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to send a soft reset
  * @param  PortNum The current port number
  * @param  SOPType SOP Type based on @ref USBPD_SOPType_TypeDef
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestSoftReset(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_SOFT_RESET, SOPType);
}

/**
  * @brief  Request the PE to send a Source Capability message.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestSourceCapability(uint8_t PortNum)
{
  /* PE will directly get the PDO saved in structure @ref PWR_Port_PDO_Storage */
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_SRC_CAPABILITIES, NULL, 0);
}


/**
  * @brief  Request the PE to send an ALERT to port partner
  * @param  PortNum The current port number
  * @param  Alert   Alert based on @ref USBPD_ADO_TypeDef
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum, USBPD_ADO_TypeDef Alert)
{
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_ALERT, (uint32_t*)&Alert.d32, 1);
}

/**
  * @brief  Request the PE to get a source capability extended
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapabilityExt(uint8_t PortNum)
{
  return USBPD_ERROR;
}

/**
  * @brief  Request the PE to get a manufacturer infor
  * @param  PortNum The current port number
  * @param  SOPType SOP Type
  * @param  pManuInfoData Pointer on manufacturer info based on @ref USBPD_GMIDB_TypeDef
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetManufacturerInfo(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint8_t* pManuInfoData)
{
  return USBPD_PE_SendExtendedMessage(PortNum, SOPType, USBPD_EXT_GET_MANUFACTURER_INFO, (uint8_t*)pManuInfoData, sizeof(USBPD_GMIDB_TypeDef));
}

/**
  * @brief  Request the PE to request a GET_PPS_STATUS
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetPPS_Status(uint8_t PortNum)
{
  return USBPD_ERROR;
}

/**
  * @brief  Request the PE to request a GET_STATUS
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetStatus(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_STATUS, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to perform a Fast Role Swap.
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestFastRoleSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_FR_SWAP, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to send a GET_COUNTRY_CODES message
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryCodes(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_COUNTRY_CODES, USBPD_SOPTYPE_SOP);
}

/**
  * @brief  Request the PE to send a GET_COUNTRY_INFO message
  * @param  PortNum     The current port number
  * @param  CountryCode Country code (1st character and 2nd of the Alpha-2 Country)
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryInfo(uint8_t PortNum, uint16_t CountryCode)
{
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_GET_COUNTRY_INFO, (uint32_t*)&CountryCode, 1);
}

/**
  * @brief  Request the PE to send a GET_BATTERY_CAPA
  * @param  PortNum         The current port number
  * @param  pBatteryCapRef  Pointer on the Battery Capability reference
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryCapability(uint8_t PortNum, uint8_t *pBatteryCapRef)
{
  return USBPD_ERROR;
}

/**
  * @brief  Request the PE to send a GET_BATTERY_STATUS
  * @param  PortNum           The current port number
  * @param  pBatteryStatusRef Pointer on the Battery Status reference
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum, uint8_t *pBatteryStatusRef)
{
  return USBPD_ERROR;
}

/**
  * @brief  Request the PE to send a SECURITY_REQUEST
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestSecurityRequest(uint8_t PortNum)
{
  return USBPD_OK;
}

/**
  * @brief  Request the PE to send a FIRWMARE_UPDATE
  * @param  PortNum The current port number
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestFirwmwareUpdate(uint8_t PortNum)
{
  return USBPD_OK;
}

/**
  * @}
  */


/** @addtogroup USBPD_USER_PRIVATE_FUNCTIONS
  * @{
  */

/**
  * @brief  Find PDO index that offers the most amount of power.
  * @param  PortNum Port number
  * @param  PtrRequestedPower  Sink requested power profile structure pointer
  * @retval Index of PDO within source capabilities message (-1 indicating not found)
  */
static int32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_SNKPowerRequest_TypeDef* PtrRequestedPower)
{
  uint32_t *ptpdoarray;
  USBPD_PDO_TypeDef  pdo;
  uint32_t mv = 0, voltage = 0, max_voltage = 0, curr_dist = 0, temp_dist = 0;
  uint32_t nbpdo;
  int8_t curr_index = -1, temp_index = -1;

  /* Max voltage is always limited by the board's max request */
  voltage = PtrRequestedPower->OperatingVoltageInmVunits;
  max_voltage = PtrRequestedPower->MaxOperatingVoltageInmVunits;

  /* The requested voltage not supported by this board */
  if(USBPD_IS_VALID_VOLTAGE(voltage, PtrRequestedPower->MaxOperatingVoltageInmVunits, PtrRequestedPower->MinOperatingVoltageInmVunits) != 1)
  {
    /* DPM_FindVoltageIndex: Requested voltage not supported by the board\r */
    return curr_index;
  }

  /* Search PDO index among Source PDO of Port */
  nbpdo = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO;
  ptpdoarray = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO;

  /* search the better PDO in the list of source PDOs */
  for(temp_index = 0; temp_index < nbpdo; temp_index++)
  {
    pdo.d32 = ptpdoarray[temp_index];
      /* get voltage value from PDO */
      mv = pdo.GenericPDO.VoltageIn50mVunits * 50;

      /* check if the source PDO is ok in term of voltage */
      if (mv <= max_voltage)
      {
        /* choose the "better" PDO, in this case only the distance in absolute value from the target voltage */
        temp_dist = mv > voltage ? mv - voltage : voltage - mv;
        if (curr_index == -1 || curr_dist >= temp_dist)
        {
          /* consider the current PDO the better one until this time */
          curr_index = temp_index;
          curr_dist = temp_dist;
        }
      }
  }

  return curr_index;
}

/**
  * @brief  Evaluate received Capabilities Message from Source port and prepare the request message
  * @param  PortNum           Port number
  * @param  IndexSrcPDO       Index on the selected SRC PDO (value between 1 to 7)
  * @param  RequestedVoltage  Requested voltage (in MV and use mainly for APDO)
  * @param  Rdo               Pointer on the RDO
  * @param  PtrPowerObject    Pointer on the selected power object
  * @retval None
  */
void DPM_SNK_GetSelectedPDO(uint8_t PortNum, uint8_t IndexSrcPDO, uint16_t RequestedVoltage, USBPD_SNKRDO_TypeDef* Rdo, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject)
{
  uint32_t mv = 0, mw = 0, ma = 0;
  USBPD_PDO_TypeDef  pdo;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];

  pdhandle->DPM_RequestedVoltage = 0;

  /* Initialize RDO */
  rdo.d32 = 0;

  /* Extract power information from Power Data Object */
  pdo.d32 = pdhandle->DPM_ListOfRcvSRCPDO[IndexSrcPDO];

  /* Set the Object position */
  rdo.GenericRDO.ObjectPosition = IndexSrcPDO + 1;
  *PtrPowerObject = pdo.GenericPDO.PowerObject;

  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo = pdo.SRCFixedPDO;
      mv = PWR_DECODE_50MV(fixedpdo.VoltageIn50mVunits);
      ma = PWR_DECODE_10MA(fixedpdo.MaxCurrentIn10mAunits);
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      mw = ma * mv; /* mW */
      DPM_Ports[PortNum].DPM_RequestedCurrent = ma;
      rdo.FixedVariableRDO.OperatingCurrentIn10mAunits  = ma / 10;
      rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = ma / 10;
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
        rdo.FixedVariableRDO.CapabilityMismatch = 1;
        /* USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power */
      }
    }
    break;
  case USBPD_CORE_PDO_TYPE_BATTERY:
    {
      USBPD_SRCBatterySupplyPDO_TypeDef batterypdo = pdo.SRCBatteryPDO;
      mv = PWR_DECODE_50MV(batterypdo.MinVoltageIn50mVunits);
      mw = PWR_DECODE_MW(batterypdo.MaxAllowablePowerIn250mWunits);
      mw = USBPD_MIN(mw, puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits); /* mW */
      ma = mw / mv; /* mA */
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      DPM_Ports[PortNum].DPM_RequestedCurrent       = ma;
      mw = ma * mv; /* mW */
      rdo.BatteryRDO.ObjectPosition                 = IndexSrcPDO + 1;
      rdo.BatteryRDO.OperatingPowerIn250mWunits     = mw / 250;
      rdo.BatteryRDO.MaxOperatingPowerIn250mWunits  = mw / 250;
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        rdo.BatteryRDO.CapabilityMismatch = 1;
        /* Mismatch, less power offered than the operating power */
      }
      /* USBPD_DPM_EvaluateCapabilities: Battery Request Data Object %u: %d mW", rdo.d32, mw */
    }
    break;
  case USBPD_CORE_PDO_TYPE_VARIABLE:
    {
      //USBPD_SRCVariableSupplyPDO_TypeDef variablepdo = pdo.SRCVariablePDO;
    }
    break;
  default:
    break;
  }

  pdhandle->DPM_RequestDOMsg = rdo.d32;
  pdhandle->DPM_RDOPosition = rdo.GenericRDO.ObjectPosition;

  Rdo->d32 = pdhandle->DPM_RequestDOMsg;
  /* Get the requested voltage */
  pdhandle->DPM_RequestedVoltage = mv;
}

/**
  * @brief  Turn Off power supply.
  * @param  PortNum The current port number
  * @retval USBPD_OK, USBPD_ERROR
  */
static USBPD_StatusTypeDef DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  USBPD_StatusTypeDef status = USBPD_OK;

  status = USBPD_PWR_IF_VBUSDisable(PortNum);
  if(USBPD_PORTPOWERROLE_SRC == Role)
  {
    /* wait until discharge vbus */
    USBPD_DPM_WaitForTime(30);
  }
  return status;
}

/**
  * @brief  Turn On power supply.
  * @param  PortNum The current port number
  * @retval USBPD_ACCEPT, USBPD_WAIT, USBPD_REJECT
  */
static USBPD_StatusTypeDef DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  USBPD_StatusTypeDef status = USBPD_OK;
  /* Enable the output */
  status = USBPD_PWR_IF_VBUSEnable(PortNum);
  if(USBPD_PORTPOWERROLE_SRC == Role)
  {
    /* Enable the output */
    USBPD_DPM_WaitForTime(30);
  }
  else
  {
    /* stop current sink */
  }


  return status;
}

/**
  * @brief  Assert Rp resistor.
  * @param  PortNum The current port number
  * @retval None
  */
static void DPM_AssertRp(uint8_t PortNum)
{
  USBPD_TCPM_set_cc(PortNum, TYPEC_CC_RP, USBPD_ENABLE);

}

/**
  * @brief  Assert Rd resistor.
  * @param  PortNum The current port number
  * @retval None
  */
static void DPM_AssertRd(uint8_t PortNum)
{
  USBPD_TCPM_set_cc(PortNum, TYPEC_CC_RD, USBPD_ENABLE);

}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* check to avoid count before OSKernel Start */
  if (uxTaskGetNumberOfTasks() != 0)
  {
    if (ALERT_PORT0_GPIO_PIN == GPIO_Pin)
    {
      UBaseType_t uxReturn = uxQueueMessagesWaitingFromISR(AlarmMsgBox);
      if (uxReturn < TCPM_ALARMBOX_MESSAGES_MAX)
      {
        osMessagePut(AlarmMsgBox, (USBPD_PORT_0 << 8 | 0), osWaitForever);
      }
      else
        while(1);
    }
  }
}
static uint32_t CheckDPMTimers(void)
{
  uint32_t _timing = osWaitForever;
  uint32_t _current_timing;

  /* Calculate the minimum timers to wake-up DPM tasks */
  _current_timing = DPM_Ports[USBPD_PORT_0].DPM_TimerAlert & DPM_TIMER_READ_MSK;
  if(_current_timing > 0)
  {
    if (_current_timing < _timing)
    {
      _timing = _current_timing;
    }
  }

  return _timing;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
