/**
  ******************************************************************************
  * @file    cli_commands.c
  * @author  System Lab
  * @version V0.4.0
  * @date    17-Jan-2017
  * @brief   CLI Commands defintion and implementation.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "cli_api.h"

#include "usbpd_conf.h"
#include "usbpd_dpm.h"
#include "usbpd_pe.h"

#ifdef USBPD_CLI

/* Includes ------------------------------------------------------------------*/
#include "cli_commands.h"
#include "usbpd_def.h"
#include "usbpd_dpm.h"
#include "usbpd_pe.h"
#include "usbpd_cad.h"


#if USBPD_PORT_COUNT == 1 
#define PORT_PARAM_ENABLE 0
#else
#define PORT_PARAM_ENABLE 1
#endif  /* USBPD_PORT_COUNT */
#define USBPD_DEF_PORT 0
/* External vars -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MSG_NOT_IMPLEMENTED " not implemented!\r\n"
#define CLI_PORTNUM_INVALID 0xFF
/* Private variables ---------------------------------------------------------*/
/** @defgroup CLI_Commands_Private_Variable CLI Commands Private Variables
 * @{
 */
/** 
 * brief  handle of the thread
 */
osThreadId xCmdThreadId;

/** 
 * brief  receive in this queue the command (without endline)
 */
static osMessageQId xQueueIn = NULL;
/** 
 * brief  send to this queue the output generated
 */
static osMessageQId xQueueOut = NULL;

/** 
 * brief  Buffer for the input string
 */
static char cInputString[ CLI_INPUT_MAX_SIZE ];
/** 
 * 
 */
static uint32_t cSNKPDONum = 0;
static uint32_t aSNKPDOBuffer[USBPD_MAX_NB_PDO];
static uint8_t cSNKPDOType = 0;
static uint32_t cSRCPDONum = 0;
static uint32_t aSRCPDOBuffer[USBPD_MAX_NB_PDO];
static uint8_t cSRCPDOType = 0;
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
static void prvCommandThread( void const * argument );
#if PORT_PARAM_ENABLE == 1
static portCHAR prvCommandCheckPortNumber(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
#endif /* PORT_PARAM_ENABLE */
static inline void prvGetVoltageCurrentFromPDO(uint32_t PdoValue, float *pVoltage, float *pCurrent);

static BaseType_t prvWelcomeCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvProfilesCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvStatusCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvRequestCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvPRSwapCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Commands structure definition ---------------------------------------------*/
/** @defgroup CLI_Commands_Definition CLI Commands Definition
 * @{
 */
/** 
 * brief  Welcome command definition
 */
static const CLI_Command_Definition_t xWelcomeCommand =
{
	"welcome",
	"w | welcome : Print out the welcome message\r\n",
	prvWelcomeCommandFunc, /* The function to run. */
	0 /* No parameter is expected. */
};
static const CLI_Command_Definition_t xWelcomeCommand2 =
{
	"w",
	"",
	prvWelcomeCommandFunc, /* The function to run. */
	0 /* No parameter is expected. */
};
/** 
 * brief  Profiles command definition
 */
static const CLI_Command_Definition_t xProfilesCommand =
{
	"profiles",
#if PORT_PARAM_ENABLE == 1        
	"p | profiles <port>: show the available profiles for the port\r\n",
#else
	"p | profiles : show the available profiles \r\n",
#endif /* PORT_PARAM_ENABLE */
	prvProfilesCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};
static const CLI_Command_Definition_t xProfilesCommand2 =
{
	"p",
	"",
	prvProfilesCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};

/** 
 * brief  Status command definition
 */
static const CLI_Command_Definition_t xStatusCommand =
{
	"status",
#if PORT_PARAM_ENABLE == 1        
	"s | status <port> : show the status of the PD comm for the port\r\n",
#else
	"s | status : show the status of the PD comm\r\n",
#endif /* PORT_PARAM_ENABLE */
	prvStatusCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};
static const CLI_Command_Definition_t xStatusCommand2 =
{
	"s",
	"",
	prvStatusCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};
/** 
 * brief  Request command definition
 */
static const CLI_Command_Definition_t xRequestCommand =
{
	"request",
#if PORT_PARAM_ENABLE == 1        
	"r | request <port> <profile> : change PD profile (only sink) for the port\r\n",
#else
	"r | request <profile> : change PD profile (only consumer)\r\n",
#endif /* PORT_PARAM_ENABLE */
	prvRequestCommandFunc, /* The function to run. */
	(PORT_PARAM_ENABLE+1) 
};
static const CLI_Command_Definition_t xRequestCommand2 =
{
	"r",
	"",
	prvRequestCommandFunc, /* The function to run. */
	(PORT_PARAM_ENABLE+1) 
};

/** 
 * brief  Request command definition
 */
static const CLI_Command_Definition_t xPRSwapCommand =
{
	"prswap",
#if PORT_PARAM_ENABLE == 1        
	"x | prswap <port> : start a power role swap for the port\r\n",
#else
	"x | prswap : start a power role swap\r\n",
#endif /* PORT_PARAM_ENABLE */
	prvPRSwapCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};

static const CLI_Command_Definition_t xPRSwapCommand2 =
{
	"x",
	"",
	prvPRSwapCommandFunc, /* The function to run. */
	PORT_PARAM_ENABLE 
};
/**
 * @}
 */

/**
 * @brief  This function registers the commands in the FreeRTOS-CLI.
 */
void CLI_RegisterCommands( void )
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand( &xWelcomeCommand );
	FreeRTOS_CLIRegisterCommand( &xProfilesCommand );
	FreeRTOS_CLIRegisterCommand( &xStatusCommand );
	FreeRTOS_CLIRegisterCommand( &xRequestCommand );
	FreeRTOS_CLIRegisterCommand( &xPRSwapCommand );

	FreeRTOS_CLIRegisterCommand( &xWelcomeCommand2 );
	FreeRTOS_CLIRegisterCommand( &xProfilesCommand2 );
	FreeRTOS_CLIRegisterCommand( &xStatusCommand2 );
	FreeRTOS_CLIRegisterCommand( &xRequestCommand2 );
	FreeRTOS_CLIRegisterCommand( &xPRSwapCommand2 );
}

/** @defgroup CLI_Commands_Callbacks CLI Commands Callbacks
 * @{
 */

/**
 * @brief  CLI callback for the Welcome command.
 */
static BaseType_t prvWelcomeCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* To storage the status of the send */
  static unsigned char cStep = 0;
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );

  /* reset the count */
  if (cStep >= CLI_WELCOME_MESSAGE_LEN) 
  {
    cStep = 0;
  }
  
  /* Copy welcome string in the output */
  sprintf( pcWriteBuffer, CLI_ConfigWelcomeMessage[cStep++]);
  return cStep < CLI_WELCOME_MESSAGE_LEN ? pdTRUE : pdFALSE; /* to manage the iteraction */
}

/**
 * @brief  CLI callback for the profiles command.
 */
static BaseType_t prvProfilesCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static unsigned char cPort = USBPD_DEF_PORT;
  static unsigned char cSection = 0xFF; /* 0 as source, 1 as sink */
  static unsigned char cIndex = 0xFF;

  static int8_t cConnectionStatus = 0;
  static USBPD_PortPowerRole_TypeDef cRole = USBPD_PORTPOWERROLE_UNKNOWN;
  static uint8_t cCondAsSink = 0;
  float voltage = 0, current = 0;
  
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );

#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */
  
  /* get the available profiles */
  if (cSection == 0xFF)
  {
    /* show the pdo message type provider / consumer */
    cIndex = 0xFF; /* used to print the title */
    cSection = 0;
    cRole = USBPD_PORTPOWERROLE_UNKNOWN;
    cConnectionStatus = 0;

    USBPD_PE_CLI_GetCurrentRole(cPort, &cRole, &cConnectionStatus);
    cCondAsSink = cRole == USBPD_PORTPOWERROLE_SNK;
    
    /* get the profiles according to the connection status */
    switch(cConnectionStatus)
    {
    case 0: /* Unplugged */
      if (cRole == USBPD_PORTPOWERROLE_SNK || cRole == USBPD_PORTPOWERROLE_SRC)
      {
        strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
        strcat(pcWriteBuffer, " role Unplugged\r\n");
        cSRCPDOType = cCondAsSink ? 0 : 1; /* NO_SRC_PDO : MY_SRC_PDO */
        cSNKPDOType = cCondAsSink ? 1 : 0; /* MY_SNK_PDO : NO_SNK_PDO */
      }
      else /* DRP */
      {
        strcpy(pcWriteBuffer, "DRP role Unplugged\r\n");
        cSRCPDOType = 1; /* MY_SRC_PDO */
        cSNKPDOType = 1; /* MY_SNK_PDO */        
      }

      break;
    case 1: /* Type-C only */
      strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
      strcat(pcWriteBuffer, " role Type-C only\r\n");
      cSRCPDOType = cCondAsSink ? 0 : 1; /* MY_SRC_PDO */
      cSNKPDOType = cCondAsSink ? 1 : 0; /* MY_SNK_PDO */
      break;
    case 2: /* Explicit Contract Done */
      strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
      strcat(pcWriteBuffer, " role - Explicit contract\r\n");
        /* as Source in explicit contract it shows the MY_SRC_PDO and RX_SNK_PDO */
      cSRCPDOType = cCondAsSink ? 2 : 1; /* RX_SRC_PDO : MY_SRC_PDO */
      cSNKPDOType = cCondAsSink ? 1 : 2; /* MY_SNK_PDO : RX_SNK_PDO */
      break;
    default:
      strcpy( pcWriteBuffer, "Error: unknown connection status\r\n");
      cSRCPDOType = 1; /* MY_SRC_PDO */
      cSNKPDOType = 1; /* MY_SNK_PDO */
//      cSRCPDOType = 0; /* NO_SRC_PDO */
//      cSNKPDOType = 0; /* NO_SNK_PDO */
//      cIndex == 0xFF;
//      cSection = 0xFF;
      break;
    }

    cSRCPDONum = 0;
    memset(aSRCPDOBuffer, 0x00, sizeof(aSRCPDOBuffer));
    if (cSRCPDOType > 0)
    {
      USBPD_PE_CLI_GetDataInfo(cPort, cSRCPDOType == 1 ? USBPD_CORE_DATATYPE_SRC_PDO : USBPD_CORE_DATATYPE_RCV_SRC_PDO, aSRCPDOBuffer, &cSRCPDONum);
    }
    cSNKPDONum = 0;
    memset(aSNKPDOBuffer, 0x00, sizeof(aSNKPDOBuffer));
    if (cSNKPDOType > 0)
    {
      USBPD_PE_CLI_GetDataInfo(cPort, cSNKPDOType == 1 ? USBPD_CORE_DATATYPE_SNK_PDO : USBPD_CORE_DATATYPE_RCV_SNK_PDO, aSNKPDOBuffer, &cSNKPDONum);
    }
    
    return cSection == 0xFF ? pdFALSE : pdTRUE; //continue if ok
  } /* PDO calculation */

  if (cSection == 0)
  {
    /* print all my/rx source PDOs */
    strcpy(pcWriteBuffer, "");
    /* if 0 skip and move in the next section */
    if (cSRCPDOType != 0) 
    {
      /* first time print the title */
      if (cIndex == 0xFF)
      {
        strcpy(pcWriteBuffer, cSRCPDOType == 1 ? "Local" : "Received");
        strcat(pcWriteBuffer, " Source PDOs:\r\n");
        if (cSRCPDONum == 0)
        {
          strcat(pcWriteBuffer, " no PDO available\r\n");
        }
        else
        {
          cIndex = 0x00;
        }
      }
      else 
      {
        /* check if there are PDOs to print */
        if (cIndex >= cSRCPDONum || cIndex >= USBPD_MAX_NB_PDO)
        {
          /* completed */
          cIndex = 0xFF;
        }
        else
        {
          /* print the current cIndex */
          prvGetVoltageCurrentFromPDO(aSRCPDOBuffer[cIndex], &voltage, &current);
          sprintf( pcWriteBuffer, "%d) %d.%.2dV %d.%.2dA\r\n", cIndex+1,
        		  (uint16_t)voltage, ((uint16_t)((uint16_t)(voltage*100)%100)),
        		  (uint16_t)current, ((uint16_t)((uint16_t)(current*100)%100)));
          cIndex++;
        }
      }
    }
    
    if (cIndex == 0xFF)
    {
      /* move to next section*/
      cSection++;
    }
  }
  else if (cSection == 1)
  {
    /* print all my/rx sink PDOs */
    strcpy(pcWriteBuffer, "");
    /* if 0 skip and move in the next section */
    if (cSNKPDOType != 0) 
    {
      /* first time print the title */
      if (cIndex == 0xFF)
      {
        strcpy(pcWriteBuffer, cSNKPDOType == 1 ? "Local" : "Received");
        strcat(pcWriteBuffer, " Sink PDOs:\r\n");
        if (cSNKPDONum == 0)
        {
          strcat(pcWriteBuffer, " no PDO available\r\n");
        }
        else
        {
          cIndex = 0x00;
        }
      }
      else 
      {
        /* check if there are PDOs to print */
        if (cIndex >= cSNKPDONum || cIndex >= USBPD_MAX_NB_PDO)
        {
          /* completed */
          cIndex = 0xFF;
        }
        else
        {
          /* print the current cIndex */
          float voltage = 0, current = 0;
          prvGetVoltageCurrentFromPDO(aSNKPDOBuffer[cIndex], &voltage, &current);
          sprintf( pcWriteBuffer, "%d) %d.%.2dV %d.%.2dA\r\n", cIndex+1,
        		  (uint16_t)voltage, ((uint16_t)((uint16_t)(voltage*100)%100)),
        		  (uint16_t)current, ((uint16_t)((uint16_t)(current*100)%100)));
          cIndex++;
        }
      }
    }
    
    if (cIndex == 0xFF)
    {
      /* move to next section*/
      cSection++;
    }    
  }
  else
  {
    /* stop */
    cIndex = 0xFF;
    cSection = 0xFF;
    strcpy(pcWriteBuffer, "");
  }
  
  return cSection == 0xFF ? pdFALSE : pdTRUE;
}

/**
 * @brief  CLI callback for the status command.
 */

#define CONNSTATUSTEXT_COUNT 4
#define CONNSTATUSTEXT_SAFEINDEX(index) (((index) >= 0 && (index)<CONNSTATUSTEXT_COUNT) ? index : 0)
static char const * connStatusText[CONNSTATUSTEXT_COUNT] = {
  "Status unknown",
  "Unplugged",
  "Plugged Type-C only",
  "Explicit contract",
};
#define CURRROLETEXT_COUNT 4
static char const * currRoleText[CURRROLETEXT_COUNT] = {
  "Unknown",
  "Sink",
  "Source",
  "Dual Role Port",
};
static inline uint8_t prvPortPowerRole2Index(USBPD_PortPowerRole_TypeDef PortPowerRole)
{
  uint8_t ret = 0;
  switch(PortPowerRole)
  {
  case USBPD_PORTPOWERROLE_SNK:
    ret = 1;
    break;
  case USBPD_PORTPOWERROLE_SRC:
    ret = 2;
    break;
  //case USBPD_PORTPOWERROLE_DRP:
  case USBPD_PORTPOWERROLE_DRP_SNK:
  case USBPD_PORTPOWERROLE_DRP_SRC:
    ret = 3;
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

static BaseType_t prvStatusCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  static int8_t connStatus = -1;

  USBPD_PortPowerRole_TypeDef currRole = USBPD_PORTPOWERROLE_UNKNOWN;
  CCxPin_TypeDef  cc = CCNONE;
  float voltage = 0;
  uint8_t profile = 0;
  USBPD_StatusTypeDef res;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
  
  if (connStatus == -1)
  {
#if PORT_PARAM_ENABLE == 1  
  /* check the port parameter */
    cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
    /* in case of a wrong PortNumber this command */
    if (cPort == CLI_PORTNUM_INVALID)
    {
      /* reset parameter */
      connStatus = -1;
      cPort = USBPD_DEF_PORT;
      
      /* stop next call */
      return pdFALSE;
    }
#endif /* PORT_PARAM_ENABLE */

    /* print the connection status and role and eventually the value */
    USBPD_PE_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus);
    sprintf( pcWriteBuffer, "Role: %s - %s", 
            currRoleText[prvPortPowerRole2Index(currRole)], 
            connStatusText[CONNSTATUSTEXT_SAFEINDEX(connStatus+1)]
              );
  }
  else
  {
    /* Get the CC line */
    cc = USBPD_CAD_CLI_GetCCLine(cPort);

    /* manage different status */
    switch(connStatus)
    {
    case 0: /* unplugged */
      //nothing
      strcpy(pcWriteBuffer, "");
      break;
    case 1: /* Type-C only */
      sprintf(pcWriteBuffer, " CC%d", (int)cc);
      break;
    case 2: /* Explicit contract done */
      res = DPM_CLI_GetStatusInfo(cPort, &profile, &voltage, NULL);
      if (res == USBPD_OK)
      {
        sprintf(pcWriteBuffer, " CC%d Profile %d %d.%.2dV", (int)cc, profile, (uint16_t)voltage, ((uint16_t)((uint16_t)(voltage*100)%100)));
      }
      else
      {
        sprintf(pcWriteBuffer, " CC%d", (int)cc);
      }
      break;
    default: /* Unknown */
      sprintf(pcWriteBuffer, " CC%d Unknown", (int)cc);
      break;
    }
    strcat(pcWriteBuffer, "\r\n");
    connStatus = -1;
  }

  return connStatus == -1 ? pdFALSE : pdTRUE;
}

/**
 * @brief  CLI callback for the request command.
 */
static BaseType_t prvRequestCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  static portCHAR cPDOIndex = 0;
  static int8_t connStatus = -1;

  const char *pcParameter = NULL;
  BaseType_t xParameterStringLength;
  USBPD_PortPowerRole_TypeDef currRole = USBPD_PORTPOWERROLE_UNKNOWN;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  ( void ) pcParameter;

  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
        
  /* 
    Get parameters 
    if the user specify all parameters the command is a set (for the blink mode need also the period)
    otherwise the command is a get
  */
  
#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */

  USBPD_PE_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus);
  
  /* check if the cable is plugged and a contract is reached */
  if (connStatus != 2)
  {
    strcpy(pcWriteBuffer, "Request failed: no concract reached.\r\n");
    return pdFALSE;
  }

  /* the current role must be a Sink */
  if (currRole != USBPD_PORTPOWERROLE_SNK)
  {
    strcpy(pcWriteBuffer, "Request failed: allowed only for sink.\r\n");
    return pdFALSE;
  }  

  /* get the received source capabilities */
  cSRCPDONum = 0;
  memset(aSRCPDOBuffer, 0x00, sizeof(aSRCPDOBuffer));
  USBPD_PE_CLI_GetDataInfo(cPort, USBPD_CORE_DATATYPE_RCV_SRC_PDO, aSRCPDOBuffer, &cSRCPDONum);

  /* check if there is almost one source capability */
  if (cSRCPDONum == 0)
  {
    strcpy(pcWriteBuffer, "Request failed: no source caps available.\r\n");
    return pdFALSE;
  }  

  /* get the pdo index required through the command */
#if PORT_PARAM_ENABLE == 1
  pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);  
#else
  pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);  
#endif
  cPDOIndex = atoi(pcParameter);
  
  /* the current role must be a Sink */
  if (cPDOIndex == 0 || cPDOIndex > cSRCPDONum)
  {
    sprintf(pcWriteBuffer, "Request failed: wrong index param '%s' specified.\r\n", pcParameter);
    return pdFALSE;
  }  

  /* send the command through the DPM API */  
  USBPD_StatusTypeDef res = USBPD_DPM_RequestNewPowerProfile(cPort, cPDOIndex);
  if (res != USBPD_OK)
  {
    strcpy(pcWriteBuffer, "Request failed: error on command execution.\r\n");
    return pdFALSE;
  }

  float voltage = 0, current = 0;
  prvGetVoltageCurrentFromPDO(aSRCPDOBuffer[cPDOIndex-1], &voltage, &current);
  
  sprintf( pcWriteBuffer, "Requested %d : %d.%.2dV %d.%.2dA\r\n", cPDOIndex,
		  (uint16_t)voltage, ((uint16_t)((uint16_t)(voltage*100)%100)),
		  (uint16_t)current, ((uint16_t)((uint16_t)(current*100)%100)));
  
  return pdFALSE;
}

/**
 * @brief  CLI callback for the request command.
 */
static BaseType_t prvPRSwapCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  const char *pcParameter = NULL;
  int8_t connStatus = -1;
  USBPD_PortPowerRole_TypeDef currRole = USBPD_PORTPOWERROLE_UNKNOWN;

  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  ( void ) pcParameter;

  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
        
  /* 
    Get parameters 
    if the user specify all parameters the command is a set (for the blink mode need also the period)
    otherwise the command is a get
  */
  
#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */

  USBPD_PE_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus);
  
  if (connStatus == 2)
  {
    sprintf(pcWriteBuffer, "Power role swap on Port %d\r\n", cPort);
    strcat(pcWriteBuffer, "Current role: ");
    strcat(pcWriteBuffer,currRoleText[prvPortPowerRole2Index(currRole)]);
    strcat(pcWriteBuffer, "\r\n");
    USBPD_DPM_RequestPowerRoleSwap(cPort);
    osDelay(300);
    USBPD_PE_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus);
    strcat(pcWriteBuffer, "New role: ");
    strcat(pcWriteBuffer,currRoleText[prvPortPowerRole2Index(currRole)]);
    strcat(pcWriteBuffer, "\r\n");
  }
  else
  {
    strcpy(pcWriteBuffer, "Warning : power role swap not sent, missing explicit contract\r\n");
  }

  
  return pdFALSE;
}

/*-----------------------------------------------------------*/

/**
 * @brief  API To perform a start of the Commands module.
 * @param  usStackSize       specify the stack size of the task
 * @param  uxPriority        specify the priority of the task
 * @param  xQueueInParam     input queue where find the command
 * @param  xQueueOutParam    output queue where redirect the output
 */
void CLI_CommandStart( uint16_t usStackSize, 
                      osPriority xPriority, 
                      xQueueHandle xQueueInParam, 
                      xQueueHandle xQueueOutParam )
{
  CLI_RegisterCommands();
  
  xQueueIn = xQueueInParam;
  xQueueOut = xQueueOutParam;
  
  /* Create that thread that handles the console itself. */
  osThreadDef(CLICmd, prvCommandThread, xPriority, 0, usStackSize);
  xCmdThreadId = osThreadCreate(osThread(CLICmd), NULL);
  configASSERT( xCmdThreadId != NULL );

  /* Send the welcome message. */
  if (CLI_ConfigWelcomeMessage != NULL)
  {
    uint8_t x;
    for (x=0; x < CLI_WELCOME_MESSAGE_LEN; x++)
    {
      xQueueSendToBack( xQueueOut, ( signed char * ) CLI_ConfigWelcomeMessage[x], 100); 
    }
    xQueueSendToBack( xQueueOut, ( signed char * ) CLI_ConfigEndOfOutputMessage, 100); 
  }
}

/**
 * @brief  Task that perfom a command process, calling the FreeRTOS-CLI API.
 * @param  pvParameters is the pointer to the task parameter
 */
static void prvCommandThread( void const * argument )
{
  //check err queue if item
  portBASE_TYPE xStatus;
  BaseType_t xReturned;
  char *pcOutputString;
  ( void ) argument;
  
  pcOutputString = FreeRTOS_CLIGetOutputBuffer();
  
  for( ;; )
  {
    xStatus = xQueueReceive( xQueueIn, cInputString, portMAX_DELAY);
    if (xStatus == pdPASS)
    {
      if (cInputString[0] != 0)
      {
        do
        {
                /* Get the next output string from the command interpreter. */
                xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, CLI_INPUT_MAX_SIZE );

                /* Write the generated string to the UART. */
                xQueueSendToBack( xQueueOut, ( signed char * ) pcOutputString, 0); 
                //printf(pcOutputString);
        } while( xReturned != pdFALSE );
      }
      xQueueSendToBack( xQueueOut, CLI_ConfigEndOfOutputMessage, 0);
    }
  }
}
static inline void prvGetVoltageCurrentFromPDO(uint32_t PdoValue, float *pVoltage, float *pCurrent)
{
  USBPD_SRCFixedSupplyPDO_TypeDef pdo;
  pdo.d32 = PdoValue;
  
  /* convert voltage if pointer is not null */
  if (pVoltage)
  {
   *pVoltage = ((float)(pdo.b.VoltageIn50mVunits * 50) / 1000.0);
  }
  /* convert current if pointer is not null */
  if (pCurrent)
  {
    *pCurrent = ((float)(pdo.b.MaxCurrentIn10mAunits * 10) / 1000.0);
  }
}

#if PORT_PARAM_ENABLE == 1
static portCHAR prvCommandCheckPortNumber(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  portCHAR cPort = CLI_PORTNUM_INVALID;
  const char *pcParameter = NULL;
  BaseType_t xParameterStringLength;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  
  if (pcCommandString)
  {
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    cPort = pcParameter != NULL && xParameterStringLength == 1 && (pcParameter[0] >= '0' && pcParameter[0] <= ('0' + 1))  ? pcParameter[0] - '0' : 0xFF;
    if (pcWriteBuffer != NULL && cPort == CLI_PORTNUM_INVALID)
    {
      sprintf( pcWriteBuffer, "Error: invalid parameter port '%s'\r\n", pcParameter != NULL ? pcParameter : "<NULL>");
    }
  }
  return cPort;
}
#endif /* PORT_PARAM_ENABLE */
#endif /* USBPD_CLI */
