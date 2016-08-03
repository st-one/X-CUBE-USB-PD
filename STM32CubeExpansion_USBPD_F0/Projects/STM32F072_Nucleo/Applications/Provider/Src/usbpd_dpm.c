/**
  ******************************************************************************
  * @file    usbpd_dpm.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    22-June-2016
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
#include "p-nucleo-usb001.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void USBPD_CAD_Callback(uint8_t hport, USBPD_CAD_STATE State, CCxPin_TypeDef Cc);

/* List of callbacks for PF layer */
static void USBPD_DPM_HardReset(uint8_t hport);
static void USBPD_DPM_SetupNewPower(uint8_t hport, uint8_t rdoposition);
static void USBPD_DPM_ExplicitContractDone(uint8_t hport);
static void USBPD_DPM_TurnOnPower(uint8_t hport);
static void USBPD_DPM_TurnOffPower(uint8_t hport);

/* Private variables ---------------------------------------------------------*/
static USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];
USBPD_CAD_Callbacks CAD_cbs = { USBPD_CAD_Callback };
USBPD_PE_Callbacks dpmCallbacks =
{
  USBPD_DPM_SetupNewPower,
  USBPD_DPM_HardReset,
  NULL,
  USBPD_DPM_TurnOffPower,
  USBPD_DPM_TurnOnPower,
  NULL,
  NULL,
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
    DPM_Ports[index].DPM_PortPowerRole = USBPD_PORTPOWERROLE_SRC;
  }
  
  /* Set On Role Led */
  USBPDM1_LED_On(LED_PORT0_ROLE);
  /* PWR SET UP */
  USBPD_PWR_IF_Init(DPM_Ports[0].DPM_ListOfPDO,&DPM_Ports[0].DPM_NumberOfPDO);
  USBPD_PWR_IF_PowerResetGlobal();
  
  /* PE SET UP */
  USBPD_PE_Init(0,DPM_Ports[0].DPM_PortPowerRole, dpmCallbacks);
  USBPD_PE_SetPowerProfile(0,DPM_Ports[0].DPM_ListOfPDO,DPM_Ports[0].DPM_NumberOfPDO);
  
  /* CAD SET UP */
  USBPD_CAD_Init(0, USBPD_PORTPOWERROLE_SRC, CAD_cbs);
  
  /* Enable CAD on both port */
  USBPD_CAD_PortEnable(0, USBPD_CAD_ENABLE);
  
  return USBPD_OK;
}

/**
  * @brief  Main task for PE layer
  * @param  argument: Not used
  * @retval None
  */
void USBPD_DPM_Process(void)
{
    USBPD_CAD_Process(0);
    USBPD_PE_SRCProcess(0);
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
      
      /* Enable VBUS */
      USBPD_DPM_TurnOnPower(hport);
      
      /* Enable VCONN*/
      USBPD_PWR_IF_Enable_VConn(hport, Cc);
      
      DPM_Ports[hport].DPM_IsConnected = 1;
      
      /* Led feedback */
      USBPDM1_LED_On((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC));
      USBPD_PE_IsCableConnected(hport, 1);
      break;
      
    case USBPD_CAD_STATE_EMC:
      /* If this is coming from a previous connection*/
      if (DPM_Ports[hport].DPM_IsConnected)
      {
        /* Disable VBUS */
        USBPD_DPM_TurnOffPower(hport);
       
        /* Disable VCONN*/
        USBPD_PWR_IF_Enable_VConn(hport, CCNONE);

		
        /* The ufp is detached */
        USBPD_PE_IsCableConnected(hport, 0);
        DPM_Ports[hport].DPM_IsConnected = 0;
      }
      
      /* Led feedback */   
      USBPDM1_LED_On((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC));
      break;
      
    case USBPD_CAD_STATE_DETACHED:
      /* Disable VBUS */
      USBPD_DPM_TurnOffPower(hport);
      
      /* Disable VCONN*/
      USBPD_PWR_IF_Enable_VConn(hport, CCNONE);
      
      /* The ufp is detached */
      USBPD_PE_IsCableConnected(hport, 0);
      DPM_Ports[hport].DPM_IsConnected = 0;
      
      /* Led feedback */
      USBPDM1_LED_Off((hport == 0 ? LED_PORT0_CC : LED_PORT1_CC));
      USBPDM1_LED_Off((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS));
      break;
      
    case USBPD_CAD_STATE_ACCESSORY:
      break;
      
    case USBPD_CAD_STATE_DEBUG:
      break;
      
    }
  }
}


/**
  * @brief  Callback function called by PE layer when HardReset message received from PRL
    * @param  hport: The current port number
  * @retval None
  */
static void USBPD_DPM_HardReset(uint8_t hport)
{
  /* Reset the power supply */
  USBPD_DPM_TurnOffPower(hport);
}

/**
  * @brief  Turn Off power supply.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_TurnOffPower(uint8_t hport)
{
  /* Reset the power supply */
  USBPD_PWR_IF_PowerReset(hport);
  USBPDM1_LED_Off((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS));
}

/**
  * @brief  Turn On power supply.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_TurnOnPower(uint8_t hport)
{
  /* Enable the output */
  USBPD_PWR_IF_Enable(hport, ENABLE);
}

/**
    * @brief  Request the DPM to setup the new power level.
    * @param  hport: The current port number
    * @param  rdoposition: RDO position in the table of PDO (possible value from 1 to 7)
    * @retval None
  */
static void USBPD_DPM_SetupNewPower(uint8_t hport, uint8_t rdoposition)
{
  /* Check if get the right pdo position */
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
  * @brief  Callback function called by PE to inform DPM that an Explicit contract is established.
  * @param  hport: The current port number
  * @retval None
*/
static void USBPD_DPM_ExplicitContractDone(uint8_t hport)
{
  /* Turn On VBUS LED when an explicit contract is established */
  USBPDM1_LED_On((hport == 0 ? LED_PORT0_VBUS : LED_PORT1_VBUS));
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
