/**
  ******************************************************************************
  * @file    usbpd_dpm.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-June-2016
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void USBPD_CAD_Callback(uint8_t hport, USBPD_CAD_STATE State, CCxPin_TypeDef Cc);
void USBPD_PE_Task(void const *argument);
void USBPD_CAD_Task(void const *argument);

/* List of callbacks for PF layer */
static void USBPD_DPM_HardReset(uint8_t hport);
static void USBPD_DPM_SetupNewPower(uint8_t hport, uint8_t rdoposition);
static USBPD_StatusTypeDef USBPD_DPM_EvaluatPRSwap(uint8_t hport);
static void USBPD_DPM_TurnOffPower(uint8_t hport);
static void USBPD_DPM_TurnOnPower(uint8_t hport);
static void USBPD_DPM_AssertRd(uint8_t hport);
static void USBPD_DPM_AssertRp(uint8_t hport);
static void USBPD_DPM_ExplicitContractDone(uint8_t hport);
static void USBPD_DPM_RequestPowerRoleSwap(uint8_t hport);

/* Private variables ---------------------------------------------------------*/
static USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];
USBPD_CAD_Callbacks CAD_cbs = { USBPD_CAD_Callback };
osThreadId PETaskHandle;
osThreadId CADTaskHandle;
USBPD_PE_Callbacks dpmCallbacks =
{
  USBPD_DPM_SetupNewPower,
  USBPD_DPM_HardReset,
  USBPD_DPM_EvaluatPRSwap,
  USBPD_DPM_TurnOffPower,
  USBPD_DPM_TurnOnPower,
  USBPD_DPM_AssertRd,
  USBPD_DPM_AssertRp,
  USBPD_DPM_ExplicitContractDone
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize DPM for a provider port power role
  * @param  None
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_DPM_Init(void)
{
  uint32_t index = 0;
  
  memset(&DPM_Ports,0,sizeof(USBPD_HandleTypeDef));
  
  for (index = 0; index<USBPD_PORT_COUNT; index++)
  {
    DPM_Ports[index].PortPowerRole = USBPD_PORTPOWERROLE_DRP_SRC;
  }
  
  /* Led management initialization */
#ifdef USBPD_LED_SERVER
  Led_Init();
  
  /* Set the power role */
  Led_Set(LED_PORT0_ROLE, LED_MODE_BLINK_ROLE_SRC, 0);
  Led_Set(LED_PORT1_ROLE, LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
  
  /* PWR SET UP */
  USBPD_PWR_IF_Init(DPM_Ports[0].ListOfPDO,&DPM_Ports[0].NumberOfPDO);
  USBPD_PWR_IF_PowerResetGlobal();
  
  /* PE SET UP */
  USBPD_PE_Init(0, DPM_Ports[0].PortPowerRole, dpmCallbacks);
  USBPD_PE_AddPowerProfile(0, DPM_Ports[0].ListOfPDO, DPM_Ports[0].NumberOfPDO);
  
  /* CAD SET UP */
  USBPD_CAD_Init(0, USBPD_PORTPOWERROLE_SRC, CAD_cbs);
  
  /* Enable CAD on both port */
  USBPD_CAD_PortEnable(0, USBPD_CAD_ENABLE);
  
  osThreadDef(CADTask, USBPD_CAD_Task, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  CADTaskHandle = osThreadCreate(osThread(CADTask), NULL);
  
  osThreadDef(PETask, USBPD_PE_Task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE * 2);
  PETaskHandle = osThreadCreate(osThread(PETask), NULL);
  
  /* Suspend PE Task*/
  osThreadSuspend(PETaskHandle);
  
  return USBPD_OK;
}


/**
  * @brief  Main task for PE layer
  * @param  argument: Not used
  * @retval None
  */
void USBPD_PE_Task(void const *argument)
{
  for(;;)
  {
    /* PE DRP process */
    USBPD_PE_DRPProcess(0);
    osDelay(2);
  }
}


/**
  * @brief  Main task for CAD layer
  * @param  argument: Not used
  * @retval None
  */
void USBPD_CAD_Task(void const *argument)
{
  for(;;)
  {
    USBPD_CAD_Process(0);
    osDelay(2);
  }
}


/**
  * @brief  CallBack reporting events on a specified port from CAD layer.
  * @param  hport: The handle of the port
  * @param  State: CAD state
  * @param  Cc:	The Communication Channel for the USBPD communication
  * @retval None
  */
void USBPD_CAD_Callback(uint8_t hport, USBPD_CAD_STATE State, CCxPin_TypeDef Cc)
{
  if ( hport == 0 )
  {
    switch(State)
    {
    case USBPD_CAD_STATE_ATTACHED:
    case USBPD_CAD_STATE_ATTEMC:
      /* An ufp is attached on the port*/
      osDelay(200);
      
      /* Enable VBUS */
      USBPD_PWR_IF_Enable(hport, ENABLE);
      
      /* Led feedback */
#ifdef USBPD_LED_SERVER
      Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
      
      /* Enable VCONN*/
      USBPD_PWR_IF_Enable_VConn(hport, Cc);
      
      DPM_Ports[hport].IsConnected = 1;
      
      /* Led feedback */
#ifdef USBPD_LED_SERVER
      Led_Set((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC) , (Cc == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
#endif /* USBPD_LED_SERVER */
      USBPD_PE_IsCableConnected(hport, 1);
      osThreadResume(PETaskHandle);
      break;
      
    case USBPD_CAD_STATE_EMC:
      /* If this is coming from a previous connection*/
      if (DPM_Ports[hport].IsConnected)
      {
        /* Disable VBUS */
        USBPD_PWR_IF_Enable(hport, DISABLE);
        
#ifdef USBPD_LED_SERVER
        Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
        
        /* Disable VCONN*/
        USBPD_PWR_IF_Enable_VConn(hport, CCNONE);
        
        osDelay(200); 
        
        /* The ufp is detached */
        USBPD_PE_IsCableConnected(hport, 0);
        osThreadSuspend(PETaskHandle);
        DPM_Ports[hport].IsConnected = 0;
      }
      
      /* Led feedback */
#ifdef USBPD_LED_SERVER
      Led_Set((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC) , (Cc == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
#endif /* USBPD_LED_SERVER */
      break;
      
    case USBPD_CAD_STATE_DETACHED:
      /* Disable VBUS */
      USBPD_PWR_IF_Enable(hport, DISABLE);
      
      /* Disable VCONN*/
      USBPD_PWR_IF_Enable_VConn(hport, CCNONE);
      
      osDelay(100);
      /* The ufp is detached */
      USBPD_PE_IsCableConnected(hport, 0);
      osThreadSuspend(PETaskHandle);
      DPM_Ports[hport].IsConnected = 0;
      
      /* Led feedback */
#ifdef USBPD_LED_SERVER      
      Led_Set((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC) , LED_MODE_OFF, 0);
      Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
      break;
      
    case USBPD_CAD_STATE_ACCESSORY:
      break;
      
    case USBPD_CAD_STATE_DEBUG:
      break;
      
    }
  }
  osDelay(10);
}


/**
  * @brief  Callback function called by PE layer when HardReset message received from PRL
    * @param  hport: The current port number
  * @retval None
  */
static void USBPD_DPM_HardReset(uint8_t hport)
{
  /* Reset the power supply */
  USBPD_PWR_IF_Enable(hport, DISABLE);
}


/**
    * @brief  Request the DPM to setup the new power level.
    * @param  hport: The current port number
    * @param  rdoposition: RDO position in the table of PDO (possible value from 1 to 7)
    * @retval None
  */
static void USBPD_DPM_SetupNewPower(uint8_t hport, uint8_t rdoposition)
{
  /* Check if get the rigth pdo position */
  if (rdoposition > 0)
  {
    USBPD_PWR_IF_SetProfile(hport,rdoposition-1);
  }
  else
  {
    /* Put it to VSafe5V */
    USBPD_PWR_IF_SetProfile(hport, 0);
  }
}


/**
  * @brief  Evaluate the swap request from PE.
  * @param  hport: The current port number
  * @retval USBPD_OK if PR_swap is possible, else USBPD_ERROR
*/
static USBPD_StatusTypeDef USBPD_DPM_EvaluatPRSwap(uint8_t hport)
{
  return USBPD_OK;
}


/**
  * @brief  Turn Off power supply.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_TurnOffPower(uint8_t hport)
{
  /* USBPD_PWR_IF_Enable(hport, DISABLE); */
  
#ifdef USBPD_LED_SERVER
  Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
}


/**
  * @brief  Turn On power supply.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_TurnOnPower(uint8_t hport)
{
  /* USBPD_PWR_IF_Enable(hport, ENABLE); */
  
  /* Led feedback */
#ifdef USBPD_LED_SERVER
  Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
}


/**
  * @brief  Assert Rp resistor.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_AssertRp(uint8_t hport)
{
  USBPD_CAD_AssertRp(hport);
  
#ifdef USBPD_LED_SERVER
  Led_Set((hport == 0 ? LED_PORT0_ROLE : LED_PORT1_ROLE) , LED_MODE_BLINK_ROLE_SRC, 0);
#endif /* USBPD_LED_SERVER */
  
  /* Set the new power role */
  DPM_Ports[hport].PortPowerRole = USBPD_PORTPOWERROLE_DRP_SRC;
}


/**
  * @brief  Assert Rd resistor.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_AssertRd(uint8_t hport)
{
  USBPD_CAD_AssertRd(hport);

#ifdef USBPD_LED_SERVER
    Led_Set((hport == 0 ? LED_PORT0_ROLE : LED_PORT1_ROLE) , LED_MODE_BLINK_ROLE_SNK, 0);
#endif /* USBPD_LED_SERVER */
  
  /* Set the new power role */
  DPM_Ports[hport].PortPowerRole = USBPD_PORTPOWERROLE_DRP_SNK;
}


/**
  * @brief  Callback function called by PE to inform DPM that an Explicit contract is established.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_ExplicitContractDone(uint8_t hport)
{
  /* Turn On VBUS LED when an explicit contract is established */
#ifdef USBPD_LED_SERVER
  Led_Set((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) , LED_MODE_ON, 0);
#endif /* USBPD_LED_SERVER */
}


/**
  * @brief  Request the PE to perform a Power Role Swap.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_RequestPowerRoleSwap(uint8_t hport)
{
  USBPD_PE_RequestPowerRoleSwap(hport);
}


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13)
  {
    USBPD_DPM_RequestPowerRoleSwap(0);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
