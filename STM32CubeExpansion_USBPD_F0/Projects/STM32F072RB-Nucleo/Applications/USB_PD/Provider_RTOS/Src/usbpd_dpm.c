/**
  ******************************************************************************
  * @file    usbpd_dpm.c
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
#include "usbpd_dpm.h"
#include "usbpd_pwr_if.h"
#include "usbpd_cad.h" 
#include "cmsis_os.h"
#include "p-nucleo-usb001.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void USBPD_CAD_Callback(uint8_t PortNum, USBPD_CAD_STATE State, CCxPin_TypeDef Cc);
void USBPD_PE_Task(void const *argument);
void USBPD_CAD_Task(void const *argument);

/* List of callbacks for PE layer */
static uint32_t USBPD_DPM_HardReset(uint8_t PortNum);
static void USBPD_DPM_SetupNewPower(uint8_t PortNum);
static void USBPD_DPM_ExplicitContractDone(uint8_t PortNum);
static void USBPD_DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role);
static void USBPD_DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef role);
static void USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t *Size);  
static void USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t Size);  
static USBPD_StatusTypeDef USBPD_DPM_EvaluateRequest(uint8_t PortNum);
static USBPD_StatusTypeDef USBPD_DPM_EvaluateCapabilities(uint8_t PortNum);


/* Private variables ---------------------------------------------------------*/
static USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];
#ifdef USBPD_LED_SERVER
static LED_Roles DPM_Leds[USBPD_PORT_COUNT] =
{
  {LED_PORT0_CC, LED_PORT0_VBUS, LED_PORT0_ROLE},
#if USBPD_PORT_COUNT >= 2
  {LED_PORT1_CC, LED_PORT1_VBUS, LED_PORT1_ROLE},
#endif
};
#endif
USBPD_CAD_Callbacks CAD_cbs = { USBPD_CAD_Callback };
osThreadId PETaskHandle;
osThreadId CADTaskHandle;
USBPD_PE_Callbacks dpmCallbacks =
{
  USBPD_DPM_SetupNewPower,
  USBPD_DPM_HardReset,
  NULL,
  USBPD_DPM_TurnOffPower,
  USBPD_DPM_TurnOnPower,
  NULL,
  NULL,
  USBPD_DPM_ExplicitContractDone,
  USBPD_DPM_GetDataInfo,
  USBPD_DPM_SetDataInfo,
  USBPD_DPM_EvaluateRequest,
  USBPD_DPM_EvaluateCapabilities,
};

/* Private functions ---------------------------------------------------------*/
USBPD_StatusTypeDef USBPD_DPM_RequestNewPowerProfile(uint8_t PortNum, uint8_t PDOIndex);
static int32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_SNKPowerRequest_TypeDef* PtrRequestedPower);
USBPD_StatusTypeDef DPM_SetSNKRequiredPower(uint8_t PortNum, uint32_t Current, uint32_t Voltage, uint32_t MaxVoltage, uint32_t MinVoltage);

/**
  * @brief  Initialize DPM (port power role, PWR_IF, CAD and PE Init procedures)
  * @param  None
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_DPM_Init(void)
{
  uint32_t index = 0;

  memset(&DPM_Ports, 0, sizeof(USBPD_HandleTypeDef)*USBPD_PORT_COUNT);

  for (index = 0; index<USBPD_PORT_COUNT; index++)
  {
    DPM_Ports[index].DPM_PortPowerRole = USBPD_PORTPOWERROLE_SRC;
    DPM_Ports[index].DPM_NumberOfRcvSRCPDO = 0;
    DPM_Ports[index].DPM_NumberOfRcvSNKPDO = 0;
    DPM_Ports[index].DPM_RcvRequestDOMsg = 0;
    DPM_Ports[index].DPM_RDOPosition = 0;
    DPM_Ports[index].DPM_RequestDOMsg = 0;
    DPM_Ports[index].DPM_RequestedVoltage = 0;
    USBPD_PWR_IF_SetRole(index, DPM_Ports[index].DPM_PortPowerRole);
  }
  
  /* Led management initialization */
#ifdef USBPD_LED_SERVER
  Led_Init();
  
  /* Set the power role */
  Led_Set(LED_PORT0_ROLE, LED_MODE_BLINK_ROLE_SRC, 0);
  Led_Set(LED_PORT1_ROLE, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
  
  /* PWR SET UP */
  USBPD_PWR_IF_Init();
  USBPD_PWR_IF_PowerResetGlobal();
  
  /* PE SET UP : Port 0 */
  USBPD_PE_Init(USBPD_PORT_0, DPM_Ports[USBPD_PORT_0].DPM_PortPowerRole, dpmCallbacks);

  /* CAD SET UP : Port 0 */
  USBPD_CAD_Init(USBPD_PORT_0, CAD_cbs);

  /* Enable CAD on Port 0 */
  USBPD_CAD_PortEnable(USBPD_PORT_0, USBPD_CAD_ENABLE);
  
  osThreadDef(CADTask, USBPD_CAD_Task, osPriorityLow, 0, configMINIMAL_STACK_SIZE * 2);
  CADTaskHandle = osThreadCreate(osThread(CADTask), NULL);
  
  /* PE task to be created on attachment */
  PETaskHandle = NULL;
  
  return USBPD_OK;
}

/**
  * @brief  Main task for PE layer
  * @param  argument Not used
  * @retval None
  */
void USBPD_PE_Task(void const *argument)
{
  for(;;)
  {
    USBPD_PE_SRCProcess(USBPD_PORT_0);
    osDelay(2);
  }
}


/**
  * @brief  Main task for CAD layer
  * @param  argument Not used
  * @retval None
  */
void USBPD_CAD_Task(void const *argument)
{
  for(;;)
  {
    USBPD_CAD_Process();
    osDelay(CAD_Task_delay);
  }
}


/**
  * @brief  CallBack reporting events on a specified port from CAD layer.
  * @param  PortNum The handle of the port
  * @param  State CAD state
  * @param  Cc	The Communication Channel for the USBPD communication
  * @retval None
  */
void USBPD_CAD_Callback(uint8_t PortNum, USBPD_CAD_STATE State, CCxPin_TypeDef Cc)
{
  assert_param(USBPD_PORT_IsValid(PortNum));
  
  switch(State)
  {
  case USBPD_CAD_STATE_ATTACHED:
  case USBPD_CAD_STATE_ATTEMC:
    /* An sink is attached on the port */
    osDelay(100);
    
    /* Enable VBUS */
    USBPD_DPM_TurnOnPower(PortNum, PE_GetPowerRole(PortNum));
    
    /* Enable VCONN*/
    USBPD_PWR_IF_Enable_VConn(PortNum, Cc);
    
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    
    /* Led feedback */
#ifdef USBPD_LED_SERVER
    Led_Set(DPM_Leds[PortNum].CCLine, LED_MODE_BLINK_CC(Cc), 0);
    Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
    osDelay(150);
    
    USBPD_PE_IsCableConnected(PortNum, 1);
    /* Create PE task */
    if (PETaskHandle == NULL)
    {
      osThreadDef(PETask, USBPD_PE_Task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE * 2);
      PETaskHandle = osThreadCreate(osThread(PETask), &PortNum);
    }
    break;
    
  case USBPD_CAD_STATE_EMC:
    /* If this is coming from a previous connection*/
    if (DPM_Ports[PortNum].DPM_IsConnected)
    {
      /* Disable VBUS */
      USBPD_DPM_TurnOffPower(PortNum, PE_GetPowerRole(PortNum));
      
      /* Disable VCONN*/
      USBPD_PWR_IF_Enable_VConn(PortNum, CCNONE);
      
      osDelay(200);
      
      /* The ufp is detached */
      USBPD_PE_IsCableConnected(PortNum, 0);
      if (PETaskHandle != NULL)
      {
        osThreadTerminate(PETaskHandle);
        PETaskHandle = NULL;
      }
      DPM_Ports[PortNum].DPM_IsConnected = 0;
    }
    
    /* Led feedback */
#ifdef USBPD_LED_SERVER
    Led_Set(DPM_Leds[PortNum].CCLine, LED_MODE_BLINK_CC(Cc), 0);
#endif /* USBPD_LED_SERVER */
    break;
    
  case USBPD_CAD_STATE_DETACHED:
    /* Disable VBUS */
    USBPD_DPM_TurnOffPower(PortNum,PE_GetPowerRole(PortNum));
    
    /* Disable VCONN*/
    USBPD_PWR_IF_Enable_VConn(PortNum, CCNONE);
    
    /* a delay to be sure of the turn power off */
    osDelay(100);
    /* The sink is detached */
    USBPD_PE_IsCableConnected(PortNum, 0);
    
    /* PE Task termination */
    if (PETaskHandle != NULL)
    {
      osThreadTerminate(PETaskHandle);
      PETaskHandle = NULL;
    }
    DPM_Ports[PortNum].DPM_IsConnected = 0;
    
    /* Led feedback */
#ifdef USBPD_LED_SERVER      
    Led_Set(DPM_Leds[PortNum].CCLine, LED_MODE_OFF, 0);
    Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
    break;
    
  case USBPD_CAD_STATE_ACCESSORY:
    break;
    
  case USBPD_CAD_STATE_DEBUG:
    break;
    
  default:
    break;
  }
}


/**
  * @brief  Callback function called by PE layer when HardReset message received from PRL
  * @param  PortNum The current port number
  * @retval None
  */
static uint32_t USBPD_DPM_HardReset(uint8_t PortNum)
{
  if(USBPD_PORTPOWERROLE_SRC == PE_GetPowerRole(PortNum))
  {
    /* Reset the power supply */
    USBPD_DPM_TurnOffPower(PortNum,PE_GetPowerRole(PortNum));
    return 1;
  }
  else
  {
    return USBPD_PWR_IF_IsEnabled(PortNum);
  }
}


/**
  * @brief  Turn Off power supply.
  * @param  PortNum The current port number
  * @retval None
  */
static void USBPD_DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  /* Reset the power supply */
  USBPD_PWR_IF_PowerReset(PortNum);

  osDelay(30); /* wait until discharge vbus */

#ifdef USBPD_LED_SERVER
  Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
}

/**
  * @brief  Turn On power supply.
  * @param  PortNum The current port number
  * @retval None
  */
static void USBPD_DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  /* Enable the output */
  USBPD_PWR_IF_Enable(PortNum, ENABLE, Role);
  osDelay(30);
  /* Led feedback */
#ifdef USBPD_LED_SERVER
  Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
}

/**
  * @brief  Request the DPM to setup the new power level.
  * @param  PortNum The current port number
  * @retval None
  */
static void USBPD_DPM_SetupNewPower(uint8_t PortNum)
{
  uint8_t rdoposition;

  /* Retrieve Request DO position from DPM handle : RDO position in the table of PDO (possible value from 1 to 7) */
  rdoposition = DPM_Ports[PortNum].DPM_RDOPosition;

  /* Check if get the right pdo position */
  if (rdoposition > 0)
  {
    USBPD_PWR_IF_SetProfile(PortNum, rdoposition-1);
  }
  else
  {
    /* Put it to VSafe5V */
    USBPD_PWR_IF_SetProfile(PortNum, 0);
  }
}

/**
  * @brief  Callback function called by PE to inform DPM that an Explicit contract is established.
  * @param  PortNum The current port number
  * @retval None
  */
static void USBPD_DPM_ExplicitContractDone(uint8_t PortNum)
{
  /* Turn On VBUS LED when an explicit contract is established */
#ifdef USBPD_LED_SERVER
  Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_ON, 0);
#endif /* USBPD_LED_SERVER */
}

/**
  * @brief  DPM callback to allow PE to retrieve information from DPM/PWR_IF.
  * @param  PortNum Port number
  * @param  DataId Type of data to be read from DPM
  *         This parameter can be one of the following values:
  *           @arg @ref USBPD_CORE_DATATYPE_SRC_PDO Source PDO reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_SNK_PDO Sink PDO reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_RCV_SRC_PDO Received Source PDO values reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_RCV_SNK_PDO Received Sink PDO values reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_REQ_VOLTAGE Requested voltage value reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_REQUEST_DO  Request message DO reading requested (from Sink to Source)
  * @param  Ptr Pointer on address where DPM data should be written (u32 pointer)
  * @param  Size Pointer on nb of u32 written by DPM
  * @retval None
  */
static void USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t *Size)
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
      break;

    /* Case Port Received Source PDO Data information (from distant port) */
    case USBPD_CORE_DATATYPE_RCV_SRC_PDO : 
      for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO; index++)
      {
        *(uint32_t*)(Ptr + index) = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index];
      }
      *Size = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO;
      break;

    /* Case Port Received Sink PDO Data information (from distant port) */
    case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
      for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO; index++)
      {
        *(uint32_t*)(Ptr + index) = DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index];
      }
      *Size = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO;
      break;

    /* Case Requested voltage value Data information */
    case USBPD_CORE_DATATYPE_REQ_VOLTAGE :
      *Ptr = DPM_Ports[PortNum].DPM_RequestedVoltage;
      *Size = 1;
      break;

    /* Case Request message DO (from Sink to Source) Data information */
    case USBPD_CORE_DATATYPE_REQUEST_DO :
      *Ptr = DPM_Ports[PortNum].DPM_RequestDOMsg;
      *Size = 1;
      break;

    /* In case of unexpected data type (reading request could not be fulfilled) :
       Set nb of read u32 to 0.
    */
    default :
      *Size = 0;
      break;
  }
}

/**
  * @brief  DPM callback to allow PE to update information in DPM/PWR_IF.
  * @param  PortNum Port number
  * @param  DataId Type of data to be updated in DPM
  *         This parameter can be one of the following values:
  *           @arg @ref USBPD_CORE_DATATYPE_RDO_POSITION Storage of requested DO position in PDO list
  *           @arg @ref USBPD_CORE_DATATYPE_REQ_VOLTAGE Storage of requested voltage value
  *           @arg @ref USBPD_CORE_DATATYPE_RCV_SRC_PDO Storage of Received Source PDO values 
  *           @arg @ref USBPD_CORE_DATATYPE_RCV_SNK_PDO Storage of Received Sink PDO values
  *           @arg @ref USBPD_CORE_DATATYPE_RCV_REQ_PDO Storage of Received Sink Request PDO value
  *           @arg @ref USBPD_CORE_DATATYPE_REQUEST_DO  Storage of Request message DO (from Sink to Source)
  * @param  Ptr Pointer on address where DPM data to be updated could be read (u32 pointer)
  * @param  Size nb of u32 to be updated in DPM
  * @note   In case of USBPD_CORE_DATATYPE_RDO_POSITION or USBPD_CORE_DATATYPE_REQ_VOLTAGE 
  *         data information types, Size value is expected to be equal to 1.
  * @retval None
  */
static void USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint32_t *Ptr, uint32_t Size)
{
  uint32_t index;

  /* Check type of information targeted by request */
  switch (DataId)
  {
    /* Case requested DO position Data information :
    */
    case USBPD_CORE_DATATYPE_RDO_POSITION :
      if (Size == 1)
      {
        DPM_Ports[PortNum].DPM_RDOPosition = *Ptr;
      }
      break;

    /* Case requested Voltage Data information :
    */
    case USBPD_CORE_DATATYPE_REQ_VOLTAGE :
      if (Size == 1)
      {
        DPM_Ports[PortNum].DPM_RequestedVoltage = *Ptr;
      }
      break;

    /* Case Received Source PDO values Data information :
    */
    case USBPD_CORE_DATATYPE_RCV_SRC_PDO : 
      if (Size <= USBPD_MAX_NB_PDO)
      {
        DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO = Size;
        /* Copy PDO data in DPM Handle field */
        for (index = 0; index < Size; index++)
        {
          DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index] = LE32(Ptr + index);
        }
      }
      break;

    /* Case Received Sink PDO values Data information :
    */
    case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
      if (Size <= USBPD_MAX_NB_PDO)
      {
        DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO = Size;
        /* Copy PDO data in DPM Handle field */
        for (index = 0; index < Size; index++)
        {
          DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index] = LE32(Ptr + index);
        }
      }
      break;

    /* Case Received Request PDO Data information :
    */
    case USBPD_CORE_DATATYPE_RCV_REQ_PDO :
      if (Size == 1)
      {
        DPM_Ports[PortNum].DPM_RcvRequestDOMsg = *Ptr;
      }
      break;

    /* Case RRequest message DO (from Sink to Source) Data information :
    */
    case USBPD_CORE_DATATYPE_REQUEST_DO :
      if (Size == 1)
      {
        DPM_Ports[PortNum].DPM_RcvRequestDOMsg = *Ptr;
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
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_DPM_EvaluateRequest(uint8_t PortNum)
{
  uint32_t rdo = 0, pdo = 0, pdomaxcurrent = 0;
  uint32_t rdomaxcurrent = 0, rdoopcurrent = 0, rdoobjposition = 0;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];

  rdo = pdhandle->DPM_RcvRequestDOMsg;
  rdomaxcurrent = rdo & 0x3FF;
  rdoopcurrent = (rdo >> 10) & 0x3FF;
  rdoobjposition = (rdo >> 28) & 0x7;
  pdhandle->DPM_RDOPosition = 0;
  
  /* Check if RDP can be met within the supported PDOs by the Source port */
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Evaluate Sink Request\r");
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Check if RDP can be met within the supported PDOs by the Source port\r");
  
  /* Search PDO in Port Source PDO list, that corresponds to Position provided in Request RDO */
  if (USBPD_PWR_IF_SearchRequestedPDO(PortNum, rdoobjposition, &pdo) != USBPD_OK)
  {
    /* Invalid PDO index */
    USBPD_ErrLog("USBPD_DPM_EvaluateRequest: Invalid PDOs index\r");
    return USBPD_FAIL;
  }

  pdomaxcurrent = (pdo & 0x3FF);
  
  if(rdoopcurrent > pdomaxcurrent)
  {
    /* Sink requests too much operating current */
    USBPD_ErrLog("USBPD_DPM_EvaluateRequest: Sink requests too much operating current\r");
    return USBPD_FAIL;
  }

  if(rdomaxcurrent > pdomaxcurrent)
  {
    /* Sink requests too much maximum operating current */
    USBPD_ErrLog("USBPD_DPM_EvaluateRequest: Sink requests too much maximum operating current\r");
    return USBPD_FAIL;
  }

  /* Set RDO position and requested voltage in DPM port structure */
  pdhandle->DPM_RequestedVoltage = ((pdo >> 10) & 0x3ff) * 50;
  pdhandle->DPM_RDOPosition = rdoobjposition;

  /* Accept the requested power */
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Sink requested %d mV %d mA for operating current from %d to %d mA\r",
               ((pdo >> 10) & 0x3ff) * 50, (pdo & 0x3ff) * 10,
               ((rdo >> 10) & 0x3ff) * 10, (rdo & 0x3ff) * 10);
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Source accepts the requested power\r");

  return USBPD_OK;
}


/**
  * @brief  Evaluate received Capabilities Message from Source port and prepare the request message  
  * @param  PortNum Port number
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_DPM_EvaluateCapabilities(uint8_t PortNum)
{
  uint32_t mv = 0, mw = 0, ma = 0, pdo;
  USBPD_SNKBatteryRDO_TypeDef rdpbattery;
  USBPD_SNKFixedVariableRDO_TypeDef rdpfixed;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  int32_t pdoindex = 0;
  
  pdhandle->DPM_RequestedVoltage = 0;
  
  USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Port Partner Requests Max Voltage\r");
  
  /* Find the Pdo index for the requested voltage */    	
  pdoindex = DPM_FindVoltageIndex(PortNum, &(pdhandle->DPM_SNKRequestedPower));
  
  /* If could not find desired pdo index, then return error */
  if (pdoindex == -1)
  {
    USBPD_ErrLog("PE_EvaluateCapability: could not find desired voltage\r");
    
    rdpfixed.d32 = 0;
    rdpfixed.b.ObjectPosition = 1;
    rdpfixed.b.OperatingCurrentIn10mAunits =  pdhandle->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
    rdpfixed.b.MaxOperatingCurrent10mAunits = pdhandle->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
    rdpfixed.b.CapabilityMismatch = 1;
    USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Mismatch, could not find desired pdo index\r");
    
    pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
    return USBPD_ERROR;
  }
  
  /* Extract power information from Power Data Object */
  USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Extract power information from Power Data Object\r");
  
  pdo = pdhandle->DPM_ListOfRcvSRCPDO[pdoindex];
  
  mv = ((pdo >> 10) & 0x3FF) * 50; /* mV */
  if((pdo & USBPD_PDO_TYPE_Msk) == USBPD_PDO_TYPE_BATTERY) 
  {
    mw = (pdo & 0x3FF) * 250; /* mW */
    mw = USBPD_MIN(mw, pdhandle->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits); /* mW */
    ma = mw/mv; /* mA */
  }
  else
  {
    ma = (pdo & 0x3FF) * 10; /* mA */
  }
  ma = USBPD_MIN(ma, pdhandle->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
  mw = ma * mv; /* mW */
  
  if((pdo & USBPD_PDO_TYPE_Msk) == USBPD_PDO_TYPE_BATTERY) 
  {
    rdpbattery.d32 = 0;
    rdpbattery.b.ObjectPosition = pdoindex + 1;
    rdpbattery.b.OperatingPowerIn250mWunits = mw / 250;
    rdpbattery.b.MaxOperatingPowerIn250mWunits = mw / 250; 
    if(mw < pdhandle->DPM_SNKRequestedPower.OperatingPowerInmWunits)
    {
      rdpbattery.b.CapabilityMismatch = 1;
      USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power\r");
    }
    pdhandle->DPM_RequestDOMsg = rdpbattery.d32;
    USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Battery Request Data Object %u: %d mW", pdhandle->DPM_RequestDOMsg, mw);
  }
  else
  {
    rdpfixed.d32 = 0;
    rdpfixed.b.ObjectPosition = pdoindex + 1;
    rdpfixed.b.OperatingCurrentIn10mAunits = ma / 10;
    rdpfixed.b.MaxOperatingCurrent10mAunits = ma / 10; 
    if(mw < pdhandle->DPM_SNKRequestedPower.OperatingPowerInmWunits)
    {
      rdpfixed.b.CapabilityMismatch = 1;
      USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power\r");
    }
    pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
    USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Fixed Request Data Object %u: %d mV, %d mA", pdhandle->DPM_RequestDOMsg, mv, ma);
  }
  
  /* Get the requested voltage */
  pdhandle->DPM_RequestedVoltage = mv;
  pdhandle->DPM_SNKRDOPosition = pdoindex + 1;
  
  return USBPD_OK;
}


/**
  * @brief  Prepare sending of a new Request according to Capabilities Message from Source port
  * @param  PortNum   Index of current used port
  * @retval PDOIndex  Index of PDO within source capabilities message
  *                   PDOIndex must be from 1 to Number of received profiles
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_DPM_RequestNewPowerProfile(uint8_t PortNum, uint8_t PDOIndex)
{
  uint32_t mv = 0, mw = 0, ma = 0, pdo;
  USBPD_SNKFixedVariableRDO_TypeDef rdpfixed;
  USBPD_SNKBatteryRDO_TypeDef rdpbattery;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  
  if((PDOIndex <= pdhandle->DPM_NumberOfRcvSRCPDO) && (PDOIndex > 0))
  {
    /* Extract power information from Power Data Object */
    USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Extract power information from Power Data Object\r");
    
    pdo = pdhandle->DPM_ListOfRcvSRCPDO[PDOIndex - 1];
    
    mv = ((pdo >> 10) & 0x3FF) * 50; /* mV */
    if((pdo & USBPD_PDO_TYPE_Msk) == USBPD_PDO_TYPE_BATTERY) 
    {
      mw = (pdo & 0x3FF) * 250; /* mW */
      mw = USBPD_MIN(mw, pdhandle->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits); /* mW */
      ma = mw/mv; /* mA */
    }
    else
    {
      ma = (pdo & 0x3FF) * 10; /* mA */
    }
    ma = USBPD_MIN(ma, pdhandle->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
    mw = ma * mv; /* mW */
    
    if((pdo & USBPD_PDO_TYPE_Msk) == USBPD_PDO_TYPE_BATTERY) 
    {
      rdpbattery.d32 = 0;
      rdpbattery.b.ObjectPosition = PDOIndex;
      rdpbattery.b.OperatingPowerIn250mWunits = mw / 250;
      rdpbattery.b.MaxOperatingPowerIn250mWunits = mw / 250; 
      pdhandle->DPM_RequestDOMsg = rdpbattery.d32;
      USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Battery Request Data Object %u: %d mW", pdhandle->PE_RequestDOMsg, mw);
    }
    else
    {
      rdpfixed.d32 = 0;
      rdpfixed.b.ObjectPosition = PDOIndex;
      rdpfixed.b.OperatingCurrentIn10mAunits = ma / 10;
      rdpfixed.b.MaxOperatingCurrent10mAunits = ma / 10;
      pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
      USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Fixed Request Data Object %u: %d mV, %d mA", pdhandle->PE_RequestDOMsg, mv, ma);
    }
    
    /* Get the requested voltage */
    pdhandle->DPM_RequestedVoltage = mv;
    pdhandle->DPM_SNKRDOPosition = PDOIndex;
  }
  else
  {
    USBPD_ErrLog("USBPD_DPM_RequestNewPowerProfile: Invalid PDO index\r");
    return USBPD_ERROR;
  }
  return USBPD_OK;
}


/**
  * @brief  Find PDO index that offers the most amount of power.
  * @param  PortNum Port number
  * @param  PtrRequestedPower  Sink requested power profile structure pointer
  * @retval Index of PDO within source capabilities message (-1 indicating not found)
  */
static int32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_SNKPowerRequest_TypeDef* PtrRequestedPower)
{
  uint32_t mv = 0, index  = 0, mw = 0, ma = 0, voltage = 0;
  uint32_t nbpdo;
  uint32_t *ptpdoarray;
  
  /* Max voltage is always limited by the board's max request */
  voltage = USBPD_MIN(PtrRequestedPower->OperatingVoltageInmVunits, PtrRequestedPower->MaxOperatingVoltageInmVunits);
  
  /* The requested voltage not supported by this board */
  if(USBPD_IS_VALID_VOLTAGE(voltage, PtrRequestedPower->MaxOperatingVoltageInmVunits, PtrRequestedPower->MinOperatingVoltageInmVunits) != 1)
  {
    USBPD_ErrLog("DPM_FindVoltageIndex: Requested voltage not supported by the board\r");
    return (-1);
  }

  /* Search PDO index among Source PDO of Port */
  nbpdo = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO;
  ptpdoarray = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO;
  
  for(index = 0; index < nbpdo; index++)
  {
    mv = ((ptpdoarray[index] >> 10) & 0x3FF) * 50;
    
    /* Skip any voltage not supported by this board */
    if(USBPD_IS_VALID_VOLTAGE(mv, PtrRequestedPower->MaxOperatingVoltageInmVunits, PtrRequestedPower->MinOperatingVoltageInmVunits) != 1)
    {
      USBPD_ErrLog("DPM_FindVoltageIndex: Skip any voltage not supported by this board\r");
    }
    else
    {
      if((ptpdoarray[index] & USBPD_PDO_TYPE_Msk) == USBPD_PDO_TYPE_BATTERY) 
      {
        mw = (ptpdoarray[index] & 0x3FF) * 250000;
      }
      else
      {
        ma = (ptpdoarray[index] & 0x3FF) * 10;
        ma = USBPD_MIN(ma, PtrRequestedPower->MaxOperatingCurrentInmAunits);
        mw = ma * mv;
      }
      mw = USBPD_MIN(mw, (PtrRequestedPower->MaxOperatingPowerInmWunits * 1000));
      
      if((voltage <= mv) && (mw <= (PtrRequestedPower->MaxOperatingPowerInmWunits * 1000)))
      {
        return index;
      }     
    }
  }
  
  return (-1);
}


/**
  * @brief  Set required power by a sink port and store it in DPM Handle
  * @param  PortNum     Index of Current used port
  * @param  Current     Sink board operating Current in mA units
  * @param  Voltage     Sink board operating Voltage in mV units
  * @param  MaxVoltage  Sink board Max supported Voltage in mV units
  * @param  MinVoltage  Sink board Min supported Voltage in mV units
  * @retval USBPD status
  */
USBPD_StatusTypeDef DPM_SetSNKRequiredPower(uint8_t PortNum, uint32_t Current, uint32_t Voltage, uint32_t MaxVoltage, uint32_t MinVoltage)
{
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  
  /* Add requested power by sink board to USB PD Handle */
  pdhandle->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits = Current;
  pdhandle->DPM_SNKRequestedPower.OperatingVoltageInmVunits    = Voltage;
  pdhandle->DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits = MaxVoltage;
  pdhandle->DPM_SNKRequestedPower.MinOperatingVoltageInmVunits = MinVoltage;
  pdhandle->DPM_SNKRequestedPower.OperatingPowerInmWunits      = (Current * Voltage) / 1000;
  pdhandle->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits   = (Current * MaxVoltage) / 1000;
  
  return USBPD_OK;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
