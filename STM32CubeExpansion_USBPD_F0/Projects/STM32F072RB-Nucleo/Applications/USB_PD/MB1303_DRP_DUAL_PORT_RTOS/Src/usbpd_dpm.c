/**
  ******************************************************************************
  * @file    usbpd_dpm.c
  * @author  MCD Application Team
  * @brief   DPM file for USBPD DUAL_PORT_RTOS application
  ******************************************************************************
  * @attention
  *
  * <h2><center>Copyright (c) 2017 STMicroelectronics
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
#include "cmsis_os.h"
#include "stm32f0xx_nucleo.h"
#include "string.h"

#ifdef MB1303
#include "p-nucleo-usb002.h"
#else
#include "STUSB16xx_EVAL.h"
#endif

#include "usbpd_dpm.h"
#include "usbpd_pwr_if.h"
#include "usbpd_cad.h" 
#include "cli_api.h"

#include "STUSB1602_Driver_Conf.h"
#include "STUSB1602_Driver.h"

#include "usbpd_porthandle.h"
extern STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if 1
#define DPM_PORT0_PORTPOWERROLE USBPD_PORTPOWERROLE_DRP_SRC
#define DPM_PORT1_PORTPOWERROLE USBPD_PORTPOWERROLE_DRP_SNK
#else
#define DPM_PORT0_PORTPOWERROLE USBPD_PORTPOWERROLE_SRC
#define DPM_PORT1_PORTPOWERROLE USBPD_PORTPOWERROLE_SNK
#endif

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void USBPD_CAD_Callback(uint8_t PortNum, USBPD_CAD_STATE State, CCxPin_TypeDef Cc);
void USBPD_PE_Task(void const *argument);
void USBPD_CAD_Task(void const *argument);

/* List of callbacks for PF layer */
static uint32_t USBPD_DPM_HardReset(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status);
static uint32_t USBPD_DPM_ErrorRecovery(uint8_t PortNum);
static void USBPD_DPM_SetupNewPower(uint8_t PortNum);
static void USBPD_DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static void USBPD_DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static void USBPD_DPM_ExplicitContractDone(uint8_t PortNum);
static uint8_t USBPD_DPM_GetVBusStatus(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
static void USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t *Size);  
static void USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint32_t *Ptr, uint32_t Size);  
static USBPD_StatusTypeDef USBPD_DPM_EvaluateRequest(uint8_t PortNum);
static USBPD_StatusTypeDef USBPD_DPM_EvaluateCapabilities(uint8_t PortNum);
static void USBPD_DPM_Capability(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_CAP_Status_TypeDef Status);

#ifdef USBPD_DPM_PRS
static void USBPD_DPM_AssertRd(uint8_t PortNum);
static void USBPD_DPM_AssertRp(uint8_t PortNum);
static USBPD_StatusTypeDef USBPD_DPM_EvaluatPRSwap(uint8_t PortNum);
static void USBPD_DPM_PowerRoleSwap(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_PRS_Status_TypeDef Status);
#endif /* USBPD_DPM_PRS */

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
osThreadId CADTaskHandle;
#ifdef USBPD_DPM_PRS
USBPD_PE_Callbacks dpmCallbacks =
{
  USBPD_DPM_SetupNewPower,
  USBPD_DPM_HardReset,
  USBPD_DPM_ErrorRecovery,
  USBPD_DPM_EvaluatPRSwap,
  USBPD_DPM_TurnOffPower,
  USBPD_DPM_TurnOnPower,
  USBPD_DPM_AssertRd,
  USBPD_DPM_AssertRp,
  USBPD_DPM_ExplicitContractDone,
  USBPD_DPM_GetDataInfo,
  USBPD_DPM_SetDataInfo,
  USBPD_DPM_EvaluateRequest,
  USBPD_DPM_EvaluateCapabilities,
  USBPD_DPM_Capability,
  USBPD_DPM_PowerRoleSwap,
  USBPD_DPM_GetVBusStatus,
};
#else
USBPD_PE_Callbacks dpmCallbacks =
{
  USBPD_DPM_SetupNewPower,
  USBPD_DPM_HardReset,
  USBPD_DPM_ErrorRecovery,
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
  USBPD_DPM_Capability,
  NULL,
  USBPD_DPM_GetVBusStatus,
};
#endif /* USBPD_DPM_PRS */

#ifdef USBPD_CLI
static char cli_text[CLI_OUTPUT_MAX_SIZE];
#endif
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

  memset(&DPM_Ports, 0, sizeof(USBPD_HandleTypeDef) * USBPD_PORT_COUNT);

  DPM_Ports[USBPD_PORT_0].DPM_PortPowerRole = DPM_PORT0_PORTPOWERROLE;
#if USBPD_PORT_COUNT==2
  DPM_Ports[USBPD_PORT_1].DPM_PortPowerRole = DPM_PORT1_PORTPOWERROLE;
#endif /*USBPD_PORT_COUNT==2*/

#ifdef USBPD_LED_SERVER
  /* Led management initialization */
  Led_Init();
#endif /* USBPD_LED_SERVER */
  
  for (index = 0; index<USBPD_PORT_COUNT; index++)
  {
    DPM_Ports[index].DPM_NumberOfRcvSRCPDO = 0;
    DPM_Ports[index].DPM_NumberOfRcvSNKPDO = 0;
    DPM_Ports[index].DPM_RcvRequestDOMsg = 0;
    DPM_Ports[index].DPM_RDOPosition = 0;
    DPM_Ports[index].DPM_RDOPositionPrevious = 0;
    DPM_Ports[index].DPM_RequestDOMsg = 0;
    DPM_Ports[index].DPM_RequestDOMsgPrevious = 0;
    DPM_Ports[index].DPM_RequestedVoltage = 0;
    USBPD_PWR_IF_SetRole(index, DPM_Ports[index].DPM_PortPowerRole);

#ifdef USBPD_LED_SERVER
    /* LED according to the role */
    Led_Set(DPM_Leds[index].Role, LED_MODE_FromPowerRole(DPM_Ports[index].DPM_PortPowerRole), 0);
#endif /* USBPD_LED_SERVER */
  }

#ifdef USBPD_CLI
#ifndef HAL_UART_MODULE_ENABLED
#error "To enable the CLI you need the HAL_UART_MODULE_ENABLED"
#endif

  /* CLI enabling */
  CLI_Init(BSP_USART_GetHandle());
  CLI_Run();
#endif /* USBPD_CLI */  
  
  /* Initialize requested power for Sink Port */
  DPM_SetSNKRequiredPower(USBPD_PORT_0, USBPD_BOARD_MAX_CURRENT_MA, USBPD_BOARD_REQUESTED_VOLTAGE_MV, USBPD_BOARD_MAX_VOLTAGE_MV, USBPD_BOARD_MIN_VOLTAGE_MV);
#if USBPD_PORT_COUNT==2
  DPM_SetSNKRequiredPower(USBPD_PORT_1, USBPD_BOARD_MAX_CURRENT_MA, USBPD_BOARD_REQUESTED_VOLTAGE_MV, USBPD_BOARD_MAX_VOLTAGE_MV, USBPD_BOARD_MIN_VOLTAGE_MV);
#endif /*USBPD_PORT_COUNT==2*/

  /* PWR SET UP */
  USBPD_PWR_IF_Init();
  USBPD_PWR_IF_PowerResetGlobal();
  
  /* PE SET UP : Port 0 */
  USBPD_PE_Init(USBPD_PORT_0, DPM_Ports[USBPD_PORT_0].DPM_PortPowerRole, dpmCallbacks);
#if USBPD_PORT_COUNT == 2
  /* PE SET UP : Port 1 */
  USBPD_PE_Init(USBPD_PORT_1, DPM_Ports[USBPD_PORT_1].DPM_PortPowerRole, dpmCallbacks);
#endif /* USBPD_PORT_COUNT == 2 */
  
  /* CAD SET UP */
  USBPD_CAD_Init(USBPD_PORT_0, CAD_cbs);
#if USBPD_PORT_COUNT == 2
    /* CAD SET UP */
  USBPD_CAD_Init(USBPD_PORT_1, CAD_cbs);
#endif /* USBPD_PORT_COUNT == 2 */
    
  /* Enable CAD on Port 0 */
  USBPD_CAD_PortEnable(USBPD_PORT_0, USBPD_CAD_ENABLE);
#if USBPD_PORT_COUNT == 2
  /* Enable CAD on Port 1 */
  USBPD_CAD_PortEnable(USBPD_PORT_1, USBPD_CAD_ENABLE);
#endif /* USBPD_PORT_COUNT == 2 */
  
  /* create the CAD task */
  osThreadDef(CADTask, USBPD_CAD_Task, osPriorityLow, 0, configMINIMAL_STACK_SIZE * 2);
  CADTaskHandle = osThreadCreate(osThread(CADTask), NULL);
  
  /* PE task to be created on attachment */
  DPM_Ports[USBPD_PORT_0].PE_TaskHandle = NULL;
#if USBPD_PORT_COUNT==2
  DPM_Ports[USBPD_PORT_1].PE_TaskHandle = NULL;
#endif /* USBPD_PORT_COUNT == 2 */
  
  return USBPD_OK;
}

/**
  * @brief  Main tasks for PE layer
  * @param  argument: pointer to the PortNum
  * @retval None
  */
void USBPD_PE_Task(void const *argument)
{
  /* Get the port number passed as parameter */
  uint8_t PortNum = USBPD_PORT_0;
#if USBPD_PORT_COUNT>=2
  if (argument)
  {
    PortNum = *(uint8_t *)argument;
  }
#endif

  for(;;)
  {
    /* PE DRP process */
    USBPD_PE_DRPProcess(PortNum);
    osDelay(1);
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
  /* check the port */
  assert_param(USBPD_PORT_IsValid(PortNum));

  /* get the current role from PE */    
  USBPD_PortPowerRole_TypeDef current_role = PE_GetPowerRole(PortNum);
 
  switch(State)
  {
  case USBPD_CAD_STATE_ATTACHED:
  case USBPD_CAD_STATE_ATTEMC:

#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(DPM_Leds[PortNum].CCLine, LED_MODE_BLINK_CC(Cc), 0);
    Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_BLINK_VBUS, 0);
    Led_Set(DPM_Leds[PortNum].Role, (current_role == USBPD_PORTPOWERROLE_SRC ? LED_MODE_BLINK_ROLE_SRC : LED_MODE_BLINK_ROLE_SNK), 0);
#endif /* USBPD_LED_SERVER */

    if (current_role == USBPD_PORTPOWERROLE_SRC)
    {
      /* Goal is to delay first src_cap */
      osDelay(15);
    }
    
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    USBPD_PE_IsCableConnected(PortNum, 1);
    
    USBPD_PRL_Reset(PortNum);
    
    /* Create PE task */
    if(DPM_Ports[PortNum].PE_TaskHandle == NULL)
    {
      osThreadDef(PETaskPORT, USBPD_PE_Task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE * 3);
      DPM_Ports[PortNum].PE_TaskHandle = osThreadCreate(osThread(PETaskPORT), (void *)&PortNum);
    }

#ifdef USBPD_CLI
    /* prepare the attached output */
    sprintf(cli_text,  "\r\n*** [%d] attached on CC%d ", PortNum, Cc);
    strcat(cli_text, current_role == USBPD_PORTPOWERROLE_SRC ? "(Source)" : "(Sink)");
    if (DPM_Ports[PortNum].PE_TaskHandle == NULL)
    {
       strcat(cli_text, "(PE is not started!)"); 
    }
    strcat(cli_text, "\r\n");
    CLI_Async_Notify(cli_text);
#endif /* USBPD_CLI */
    break;
    
  case USBPD_CAD_STATE_EMC:
    break;
    
  case USBPD_CAD_STATE_DETACHED:
    if (current_role == USBPD_PORTPOWERROLE_SRC)
    { 
      /* Disable VBUS */
      USBPD_DPM_TurnOffPower(PortNum, PE_GetPowerRole(PortNum));

      /* Disable VCONN*/
      USBPD_PWR_IF_Enable_VConn(PortNum, CCNONE);

      /* a delay to be sure of the turn power off */
      osDelay(10);
    }
    
    /* The port pair  is detached */
    USBPD_PE_IsCableConnected(PortNum, 0);
    
    /* Terminate PE task */
    if (DPM_Ports[PortNum].PE_TaskHandle != NULL)
    {
      osThreadTerminate(DPM_Ports[PortNum].PE_TaskHandle);
      DPM_Ports[PortNum].PE_TaskHandle = NULL;
    }
    
    /* perform a reset of all layer in any case */
    USBPD_PE_Reset(PortNum);

    /* Set the disconnection status */
    DPM_Ports[PortNum].DPM_IsConnected = 0;
    
#ifdef USBPD_CLI
    if (State == USBPD_CAD_STATE_DETACHED)
    {
      sprintf(cli_text, "\r\n*** [%d] detached\r\n", PortNum);
      CLI_Async_Notify(cli_text);
    }
#endif /* USBPD_CLI */
    
#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(DPM_Leds[PortNum].CCLine, LED_MODE_OFF, 0);
    Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_OFF, 0);
    
    /* Check port role to initialize Role LED*/
    Led_Set(DPM_Leds[PortNum].Role, LED_MODE_FromPowerRole(DPM_Ports[PortNum].DPM_PortPowerRole), 0);
#endif /* USBPD_LED_SERVER */
    break;
    
  case USBPD_CAD_STATE_ACCESSORY:
#ifdef USBPD_CLI
    sprintf(cli_text, "\r\n*** [%d] attached: audio accessory not supported\r\n", PortNum);
    CLI_Async_Notify(cli_text);
#endif /* USBPD_CLI */    
    break;
    
  case USBPD_CAD_STATE_DEBUG:
#ifdef USBPD_CLI
    sprintf(cli_text, "\r\n*** [%d] attached: debug not supported\r\n", PortNum);
    CLI_Async_Notify(cli_text);
#endif /* USBPD_CLI */    
    break;
  default:
#ifdef USBPD_CLI
    sprintf(cli_text, "\r\n*** [%d] unknown status\r\n", PortNum);
    CLI_Async_Notify(cli_text);
#endif /* USBPD_CLI */    
    break;
  }
}

/**
  * @brief  Callback function called by PE layer when HardReset message received from PRL
  * @param  PortNum The current port number
  * @param  CurrentRole the current role
  * @param  Status status on hard reset event
  * @retval 0 if continue 1 otherwise
  */
static uint32_t USBPD_DPM_HardReset(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status)
{
  HAL_StatusTypeDef res = HAL_OK;
  switch(Status)
  {
  case USBPD_HR_STATUS_START_ACK:
    res = USBPD_HW_IF_HR_Start(PortNum, CurrentRole, ACKNOWLEDGE);
    USBPD_DPM_TurnOffPower(PortNum, CurrentRole);
    break;
  case USBPD_HR_STATUS_START_REQ:
    res = USBPD_HW_IF_HR_Start(PortNum, CurrentRole, REQUEST);
    USBPD_DPM_TurnOffPower(PortNum, CurrentRole);
    break;
  case USBPD_HR_STATUS_WAIT_VBUS_VSAFE0V:
    res = USBPD_HW_IF_HR_CheckVbusVSafe0V(PortNum, CurrentRole);
    if (Ports[PortNum].Device_cut == Cut_1)
    {
      if (res == HAL_OK && CurrentRole == USBPD_PORTPOWERROLE_SNK)
      {
        /* Workaround: To emulate VBus0Safe condition starting from VBus_Presence condition */ 
        osDelay(300);
      }
    }
    break;
  case USBPD_HR_STATUS_COMPLETED:
    res = USBPD_HW_IF_HR_End(PortNum, CurrentRole);
    USBPD_DPM_TurnOnPower(PortNum, CurrentRole);
    break;
  case USBPD_HR_STATUS_FAILED:
    res = USBPD_HW_IF_HR_End(PortNum, CurrentRole);
    break;
  default:
      break;
  }
  
  return res;
}

/**
  * @brief  Request the DPM to go to Error Recovery.
  * @param  PortNum The current port number
  * @retval None
  */
static uint32_t USBPD_DPM_ErrorRecovery (uint8_t PortNum)
{
  HAL_StatusTypeDef res = HAL_ERROR;
  res = USBPD_HW_IF_ErrorRecovery(PortNum);
  return res;
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
  * @brief  Turn Off power supply.
  * @param  PortNum The current port number
  * @retval None
  */
static void USBPD_DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  USBPD_PWR_IF_Enable(PortNum, DISABLE, Role);

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

#ifdef USBPD_LED_SERVER
  /* Led feedback */
  Led_Set(DPM_Leds[PortNum].VBus, LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
}

#ifdef USBPD_DPM_PRS
/**
  * @brief  Assert Rp resistor.
  * @param  PortNum The current port number
  * @retval None
*/
static void USBPD_DPM_AssertRp(uint8_t PortNum)
{
  //USBPD_CAD_AssertRp(PortNum);
  
#ifdef USBPD_LED_SERVER
  Led_Set(DPM_Leds[PortNum].Role, LED_MODE_BLINK_ROLE_SRC, 0);
#endif /* USBPD_LED_SERVER */
  
  /* Set the new power role */
  DPM_Ports[PortNum].DPM_PortPowerRole = USBPD_PORTPOWERROLE_DRP_SRC;
}

/**
  * @brief  Assert Rd resistor.
  * @param  PortNum: The current port number
  * @retval None
*/
static void USBPD_DPM_AssertRd(uint8_t PortNum)
{
  //USBPD_CAD_AssertRd(PortNum);

#ifdef USBPD_LED_SERVER
  Led_Set(DPM_Leds[PortNum].Role, LED_MODE_BLINK_ROLE_SNK, 0);
#endif /* USBPD_LED_SERVER */
  
  /* Set the new power role */
  DPM_Ports[PortNum].DPM_PortPowerRole = USBPD_PORTPOWERROLE_DRP_SNK;
}

/**
  * @brief  Request the PE to perform a Power Role Swap.
  * @param  PortNum: The current port number
  * @retval None
*/
void USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum)
{
  USBPD_PE_RequestPowerRoleSwap(PortNum);
}

/**
  * @brief  Evaluate the swap request from PE.
  * @param  PortNum: The current port number
  * @retval USBPD_OK if PR_swap is possible, else USBPD_ERROR
*/
static USBPD_StatusTypeDef USBPD_DPM_EvaluatPRSwap(uint8_t PortNum)
{
  return USBPD_OK;
}

/**
  * @brief  Power role swap status update
  * @param  PortNum Port number
  * @param  CurrentRole the current role
  * @param  Status status on power role swap event
  */
static void USBPD_DPM_PowerRoleSwap(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_PRS_Status_TypeDef Status)
{
  uint16_t tvbusOn_max = 0x113; /*113hex is 275decimal.*/
  uint16_t timeout_switch2src_check = 0;
  uint32_t vbus_valid_status =  0;  
  if (CurrentRole == USBPD_PORTPOWERROLE_SRC || CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    switch (Status)
    {
    case USBPD_PRS_STATUS_START_ACK:
#ifdef USBPD_LED_SERVER
      /* Led feedback */
      Led_Set(DPM_Leds[PortNum].Role, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */ 
      USBPD_HW_IF_PRS_Start(PortNum, CurrentRole, ACKNOWLEDGE);
      break;
    case USBPD_PRS_STATUS_START_REQ:
#ifdef USBPD_LED_SERVER
      /* Led feedback */
      Led_Set(DPM_Leds[PortNum].Role, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */       
      USBPD_HW_IF_PRS_Start(PortNum, CurrentRole, REQUEST);
      break;
    case USBPD_PRS_STATUS_ACCEPTED:
      STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
#ifdef USBPD_CLI
      sprintf(cli_text,  "\r\n*** [%c] PRS accepted\r\n", (PortNum + '0'));
      CLI_Async_Notify(cli_text);
#endif
      break;
    case USBPD_PRS_STATUS_REJECTED:
      STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
      USBPD_HW_IF_PRS_End(PortNum, CurrentRole); /* inform PHY that PRS ends */
#ifdef USBPD_CLI
      sprintf(cli_text,  "\r\n*** [%c] PRS reject\r\n", (PortNum + '0'));
      CLI_Async_Notify(cli_text);
#endif
      break;
    case USBPD_PRS_STATUS_WAIT:
      STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
#ifdef USBPD_CLI
      sprintf(cli_text,  "\r\n*** [%c] PRS wait\r\n", (PortNum + '0'));
      CLI_Async_Notify(cli_text);
#endif
      break;
    case USBPD_PRS_STATUS_VBUS_OFF:
      USBPD_HW_IF_PRS_Vbus_OFF(PortNum, CurrentRole);
      USBPD_DPM_TurnOffPower(PortNum, CurrentRole);
      break;
    case USBPD_PRS_STATUS_SRC_RP2RD:
      USBPD_HW_IF_PRS_Assert_Rd(PortNum, CurrentRole);
      break;
    case USBPD_PRS_STATUS_SRC_PS_READY_SENT:
      break;
    case USBPD_PRS_STATUS_SNK_PS_READY_RECEIVED:
#ifdef USBPD_CLI
      sprintf(cli_text,  "\r\n*** [%c] PS Ready received\r\n", (PortNum + '0'));
      CLI_Async_Notify(cli_text);
#endif
      break;
    case USBPD_PRS_STATUS_SNK_RD2RP:
      USBPD_HW_IF_PRS_Assert_Rp(PortNum, CurrentRole);
      do 
      {
        timeout_switch2src_check = timeout_switch2src_check + 0x001;
        osDelay(1);
        vbus_valid_status = (STUSB1602_VBUS_Valid_Get(STUSB1602_I2C_Add(PortNum)));
      }
      while  ((timeout_switch2src_check < tvbusOn_max) && (vbus_valid_status != VBUS_within_VALID_vrange));
      break;
    case USBPD_PRS_STATUS_VBUS_ON:
      USBPD_DPM_TurnOnPower(PortNum, CurrentRole);
      break;
    case USBPD_PRS_STATUS_SNK_PS_READY_SENT:
      break;
    case USBPD_PRS_STATUS_SRC_PS_READY_RECEIVED:
      break;
    case USBPD_PRS_STATUS_COMPLETED:
      USBPD_HW_IF_PRS_End(PortNum, CurrentRole);
      STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
#ifdef USBPD_LED_SERVER
      /* Led feedback */
      Led_Set(DPM_Leds[PortNum].Role, (CurrentRole == USBPD_PORTPOWERROLE_SRC ? LED_MODE_BLINK_ROLE_SRC : LED_MODE_BLINK_ROLE_SNK), 0);
#endif /* USBPD_LED_SERVER */
#ifdef USBPD_CLI
      sprintf(cli_text,  "\r\n*** [%c] PRS new role: ", (PortNum + '0'));
      strcat(cli_text, CurrentRole == USBPD_PORTPOWERROLE_SRC ? "Source" : "Sink"); /* Parameter received by the event */
      strcat(cli_text, "\r\n");
      CLI_Async_Notify(cli_text);
#endif
      break;
    case USBPD_PRS_STATUS_FAILED:
    case USBPD_PRS_STATUS_ABORTED:
#ifdef USBPD_LED_SERVER
      /* Led feedback */
      Led_Set(DPM_Leds[PortNum].Role, (CurrentRole == USBPD_PORTPOWERROLE_SRC ? LED_MODE_BLINK_ROLE_SRC : LED_MODE_BLINK_ROLE_SNK), 0);
#endif /* USBPD_LED_SERVER */
      break;
    default:
      break;
    }
  }
}
#endif /* USBPD_DPM_PRS */

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
  
#ifdef USBPD_CLI
  sprintf(cli_text,  "\r\n*** [%d] selected cap #%d\r\n", PortNum, (int) DPM_Ports[PortNum].DPM_RDOPosition);
  CLI_Async_Notify(cli_text);
#endif
  
}

/**
  * @brief  Used to check the vbus status present or not
  * @param  PortNum   Index of current used port
  * @param  CurrentRole the current role
  * @retval Return 1 if the VBus is present otherwise 0
  */
uint16_t timeout_check = 0;
uint16_t tvbusOn_max = 0x113; /*113hex is 275decimal.*/
uint8_t USBPD_DPM_GetVBusStatus(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
  {
    while ((timeout_check < tvbusOn_max) && (STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_below_UVLO_threshold))
    {
      timeout_check = timeout_check + 0x005;
      osDelay(5);
    }
    if (timeout_check >= tvbusOn_max)
    {
      timeout_check = 0;
      USBPD_DPM_ErrorRecovery(PortNum);
    }
  }
  timeout_check = 0;
  osDelay(1);
  return STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_above_UVLO_threshold ? 1 : 0;
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

  assert_param(Ptr);
  assert_param(Size);
  
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
      *Size = DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO;
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
  STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);

  /* Accept the requested power */
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Sink requested %d mV %d mA for operating current from %d to %d mA\r",
               ((pdo >> 10) & 0x3ff) * 50, (pdo & 0x3ff) * 10,
               ((rdo >> 10) & 0x3ff) * 10, (rdo & 0x3ff) * 10);
  USBPD_UsrLog("USBPD_DPM_EvaluateRequest: Source accepts the requested power\r");

  return USBPD_OK;
}

/**
  * @brief  Update the status of the capability  
  * @param  PortNum Port number
  * @retval USBPD status
  */
static void USBPD_DPM_Capability(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_CAP_Status_TypeDef Status)
{
//    DPM_Ports[PortNum].DPM_RDOPosition = (DPM_Ports[PortNum].DPM_RequestDOMsg>>28);
  switch(Status)
  {
  case USBPD_CAP_STATUS_REQUESTING:
    /* Save the PDO Index and the RDO Requested */
    DPM_Ports[PortNum].DPM_RDOPositionPrevious  = DPM_Ports[PortNum].DPM_RDOPosition;
    DPM_Ports[PortNum].DPM_RequestDOMsgPrevious = DPM_Ports[PortNum].DPM_RequestDOMsg;
    break;
  case USBPD_CAP_STATUS_ACCEPTED:
    /* Update the */
    break;
  case USBPD_CAP_STATUS_REJECTED:
    DPM_Ports[PortNum].DPM_RDOPosition  = DPM_Ports[PortNum].DPM_RDOPositionPrevious;
    DPM_Ports[PortNum].DPM_RequestDOMsg = DPM_Ports[PortNum].DPM_RequestDOMsgPrevious;
    break;
  case USBPD_CAP_STATUS_WAIT:
    DPM_Ports[PortNum].DPM_RDOPosition  = DPM_Ports[PortNum].DPM_RDOPositionPrevious;
    DPM_Ports[PortNum].DPM_RequestDOMsg = DPM_Ports[PortNum].DPM_RequestDOMsgPrevious;
    break;
  default:
    break;
  }
  DPM_Ports[PortNum].DPM_RDOPosition = (DPM_Ports[PortNum].DPM_RequestDOMsg>>28);
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
    
    pdhandle->DPM_RequestDOMsgPrevious = pdhandle->DPM_RequestDOMsg;
    pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
    pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
    pdhandle->DPM_RDOPosition = rdpfixed.b.ObjectPosition;

    /* Get the requested voltage */
    pdhandle->DPM_RequestedVoltage = 5000;
    pdhandle->DPM_SNKRDOPosition = 1;
    STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);

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
    pdhandle->DPM_RequestDOMsgPrevious = pdhandle->DPM_RequestDOMsg;
    pdhandle->DPM_RequestDOMsg = rdpbattery.d32;
    pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
    pdhandle->DPM_RDOPosition = rdpbattery.b.ObjectPosition;

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
    pdhandle->DPM_RequestDOMsgPrevious = pdhandle->DPM_RequestDOMsg;
    pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
    pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
    pdhandle->DPM_RDOPosition = rdpfixed.b.ObjectPosition;
    USBPD_UsrLog("USBPD_DPM_EvaluateCapabilities: Fixed Request Data Object %u: %d mV, %d mA", pdhandle->DPM_RequestDOMsg, mv, ma);
  }
  
  /* Get the requested voltage */
  pdhandle->DPM_RequestedVoltage = mv;
  pdhandle->DPM_SNKRDOPosition = pdoindex + 1;
  STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
  
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
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  if((PDOIndex <= pdhandle->DPM_NumberOfRcvSRCPDO) && (PDOIndex > 0))
  {
    /* Extract power information from Power Data Object */
    USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Extract power information from Power Data Object\r");
    
    pdo = 0xFF;
    if (PDOIndex <= pdhandle->DPM_NumberOfRcvSRCPDO)
    {
      pdo = pdhandle->DPM_ListOfRcvSRCPDO[PDOIndex - 1];
    }
    
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
      pdhandle->DPM_RequestDOMsgPrevious = pdhandle->DPM_RequestDOMsg;
      pdhandle->DPM_RequestDOMsg = rdpbattery.d32;
      pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
      pdhandle->DPM_RDOPosition = rdpbattery.b.ObjectPosition;

      USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Battery Request Data Object %u: %d mW", pdhandle->PE_RequestDOMsg, mw);
    }
    else
    {
      rdpfixed.d32 = 0;
      rdpfixed.b.ObjectPosition = PDOIndex;
      rdpfixed.b.OperatingCurrentIn10mAunits = ma / 10;
      rdpfixed.b.MaxOperatingCurrent10mAunits = ma / 10;
      pdhandle->DPM_RequestDOMsgPrevious = pdhandle->DPM_RequestDOMsg;
      pdhandle->DPM_RequestDOMsg = rdpfixed.d32;
      pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
      pdhandle->DPM_RDOPosition = rdpfixed.b.ObjectPosition;

      USBPD_UsrLog("USBPD_DPM_RequestNewPowerProfile: Fixed Request Data Object %u: %d mV, %d mA", pdhandle->PE_RequestDOMsg, mv, ma);
    }
    
    /* Get the requested voltage */
    pdhandle->DPM_RequestedVoltage = mv;
    pdhandle->DPM_SNKRDOPosition = PDOIndex;
    STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
    ret = USBPD_PE_RequestNewPowerProfile(PortNum, PDOIndex);
  }
  else
  {
    USBPD_ErrLog("USBPD_DPM_RequestNewPowerProfile: Invalid PDO index\r");
  }
  return ret;
}

/**
  * @brief  Find PDO index that offers the most amount of power.
  * @param  PortNum Port number
  * @param  PtrRequestedPower  Sink requested power profile structure pointer
  * @retval Index of PDO within source capabilities message (-1 indicating not found)
  */
static int32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_SNKPowerRequest_TypeDef* PtrRequestedPower)
{
  uint32_t mv = 0, voltage = 0, max_voltage = 0, curr_dist = 0, temp_dist = 0;
  uint32_t nbpdo;
  uint32_t *ptpdoarray;
  int8_t curr_index = -1, temp_index = -1;
  
  /* Max voltage is always limited by the board's max request */
  voltage = PtrRequestedPower->OperatingVoltageInmVunits;
  max_voltage = PtrRequestedPower->MaxOperatingVoltageInmVunits;
  
  /* The requested voltage not supported by this board */
  if(USBPD_IS_VALID_VOLTAGE(voltage, PtrRequestedPower->MaxOperatingVoltageInmVunits, PtrRequestedPower->MinOperatingVoltageInmVunits) != 1)
  {
    USBPD_ErrLog("DPM_FindVoltageIndex: Requested voltage not supported by the board\r");
    return curr_index;
  }

  /* Search PDO index among Source PDO of Port */
  nbpdo = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO;
  ptpdoarray = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO;

  /* search the better PDO in the list of source PDOs */  
  for(temp_index = 0; temp_index < nbpdo; temp_index++)
  {
    /* get voltage value from PDO */
    mv = ((ptpdoarray[temp_index] >> 10) & 0x3FF) * 50;
    
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
#ifdef USBPD_CLI
/**
  * @brief  
  * @param  PortNum Index of current used port
  * @param  
  * @retval None.
  */
USBPD_StatusTypeDef DPM_CLI_GetStatusInfo(uint8_t PortNum, uint8_t *pProfile, float *pVoltage, uint32_t *pCurrentPDO)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  USBPD_PortPowerRole_TypeDef role = USBPD_PORTPOWERROLE_UNKNOWN;
  if (DPM_Ports[PortNum].DPM_IsConnected)
  {
    role = PE_GetPowerRole(PortNum);
    if (role == USBPD_PORTPOWERROLE_SNK)
    {
      if (pProfile)
      {
        *pProfile = DPM_Ports[PortNum].DPM_SNKRDOPosition;
        ret = USBPD_OK;
      }
      if (pVoltage)
      {
        *pVoltage = ((float)DPM_Ports[PortNum].DPM_RequestedVoltage) / 1000.0;
        ret = USBPD_OK;
      }
      if (pCurrentPDO)
      {
        *pCurrentPDO = DPM_Ports[PortNum].DPM_RequestDOMsg;
        ret = USBPD_OK;
      }
    }
    if (role == USBPD_PORTPOWERROLE_SRC)
    {
      if (pProfile)
      {
        *pProfile = DPM_Ports[PortNum].DPM_RDOPosition;
        ret = USBPD_OK;
      }
      if (pVoltage)
      {
        *pVoltage = ((float)DPM_Ports[PortNum].DPM_RequestedVoltage) / 1000.0;
        ret = USBPD_OK;
      }
      if (pCurrentPDO)
      {
        *pCurrentPDO = DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
        ret = USBPD_OK;
      }
    }
  }
  return ret;
}
#endif /* USBPD_CLI */

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
    USBPD_DPM_RequestPowerRoleSwap(USBPD_PORT_0);
  }
  else
  {
    USBPD_HW_IF_EXTI_Callback(GPIO_Pin);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
