/**
  ******************************************************************************
  * @file    usbpd_stusb16xx_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface cad functions.
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
  
  
/* Includes ------------------------------------------------------------------*/
#include "usbpd_hw_if.h"
#include "bmc.h"
#include "string.h"
#include "usbpd_cad.h"
#include "usbpd_timersserver.h"
#include "usbpd_porthandle.h"
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"

#ifdef MB1303
#include "p-nucleo-usb002.h"
#endif

#ifdef STUSB16xx_EVAL
#include "STUSB16xx_EVAL.h"
#endif

/* Includes for LL libraries */
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"

#ifdef DBG_STUSB1602
  #include "STUSB1602_Debug.h"
  #warning "STUSB16xx_EVAL debug mode (no OS)"
#endif

uint8_t nvm_read = 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* The following definition is used to mask event interrupt and prevent the assertion of the alert bit in the ALERT_STATUS register
 * STUSB16xx_CC_DETECTION_STATUS_AL_MASK masks CC detection alerts
 * STUSB16xx_MONITORING_STATUS_AL_MASK maks monitoring alerts
 * STUSB16xx_FAULT_STATUS_AL_MASK masks fault alerts  
 */
#define STUSB16xx_STATUS_AL_MASK = STUSB16xx_MONITORING_STATUS_AL_MASK | STUSB16xx_FAULT_STATUS_AL_MASK;

/* Private macro -------------------------------------------------------------*/

#define CCXHANDLE(__CC__) ((CCxPin_TypeDef) (__CC__ +1))

/* Private variables ---------------------------------------------------------*/
extern uint8_t          RXBuffer0[];    /*!< Buffer for raw data received           */
extern uint32_t         RXData0[];      /*!< Buffer for 5b decoded data received    */
extern uint32_t         TXBuffer0[];    /*!< Buffer for data to be transmitted      */
#if (USBPD_PORT_COUNT == 2)
extern uint8_t          RXBuffer1[];
extern uint32_t         RXData1[];
extern uint32_t         TXBuffer1[];
#endif

/* Handle for the ports inside @ref USBPD_HW_IF*/
STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT] =
{
  { 0, 
    (uint8_t*)TXBuffer0, 0 /* DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t)) */,
    (uint8_t*)RXBuffer0, (uint32_t*)RXData0, PHY_MAX_RAW_SIZE,
    CCNONE,
    RESET,
    HAL_UNLOCKED,
    HAL_USBPD_PORT_STATE_RESET,
    5,
    USBPD_PORTPOWERROLE_DRP_SNK,
    0,
    DISABLE,            
    USBPD_PORTDATAROLE_UFP,
    0,   /* TxSpareBits */
  },
#if (USBPD_PORT_COUNT == 2)
   { 1, 
    (uint8_t*)TXBuffer1, 0 /* DIV_ROUND_UP(PHY_MAX_RAW_SIZE, sizeof(uint32_t)) */,
    RXBuffer1, RXData1, PHY_MAX_RAW_SIZE,
    CCNONE,
    RESET,
    HAL_UNLOCKED,
    HAL_USBPD_PORT_STATE_RESET,
    0,
    USBPD_PORTPOWERROLE_DRP_SRC, 
    0,
    DISABLE,                     
    USBPD_PORTDATAROLE_DFP,
    0,   /* TxSpareBits */
   },
#endif
};


USBPD_Init_TypeDef PortDeviceInit[USBPD_PORT_COUNT] =
{
  { 0,                                  /* Instance */
    /* Enum mismatch: PortDeviceInit.RolePower = SNK_without_accessory_supp => 2 => USBPD_PORTPOWERROLE_DRP_SNK, TO BE CHECK! (added cast to avoid warning) */
    (USBPD_PortPowerRole_TypeDef)SNK_without_accessory_supp,         /* RolePower */
    USBPD_PORTDATAROLE_UFP,             /* RoleData */
    DISABLE,                            /* VendorMessages */
    DISABLE,                            /* Ping */
    DISABLE,                            /* ExtendedMessages */
    0,                                  /* PE_SCAP_HR */
    USB_C_Current_Default,              /* CCCurrentAdvertised */
    DISABLE,                            /* DataRoleSwap */
    DISABLE,                            /* PowerRoleSwap */
    DISABLE,                            /* VConnSwap */
    DISABLE,                            /* VConnSupply */
    DISABLE,                            /* VConnDischarge */
    ENABLE,                             /* VBusDischarge */
    VConn_Ilim_350mA,                   /* VConnIlim */
    DISABLE,                            /* VConnMonitoring */
    Hi_UVLO_thr_of_4_65_V,              /* VConnThresholdUVLO */
    50,                                 /* VBusSelect by 100mV step */
    5,                                  /* VbusVShiftHigh */
    5,                                  /* VbusVShiftLow */
    DISABLE,                            /* VBusRange */
    VBUS_vSafe0V_Thr_0_6V,              /* VBusThresholdVSafe0V */
    DISABLE,                            /* VddOVLO */
    DISABLE,                            /* VddUVLO */
    7,                                  /* VBusDischargeTimeTo0V */
    10,                                 /* VBusDischargeTimeToPDO */
    DISABLE,                            /* PowerAccessoryDetection */
    DISABLE                             /* PowerAccessoryTransition */
  },
#if (USBPD_PORT_COUNT == 2)
   { 1,                                 /* Instance */
    (USBPD_PortPowerRole_TypeDef)SNK_without_accessory_supp,         /* RolePower */
    USBPD_PORTDATAROLE_UFP,             /* RoleData */
    DISABLE,                            /* VendorMessages */
    DISABLE,                            /* Ping */
    DISABLE,                            /* ExtendedMessages */
    0,                                  /* PE_SCAP_HR */
    USB_C_Current_Default,              /* CCCurrentAdvertised */
    DISABLE,                            /* DataRoleSwap */
    DISABLE,                            /* PowerRoleSwap */
    DISABLE,                            /* VConnSwap */
    DISABLE,                            /* VConnSupply */
    DISABLE,                            /* VConnDischarge */
    DISABLE,                            /* VBusDischarge */
    VConn_Ilim_350mA,                   /* VConnIlim */
    DISABLE,                            /* VConnMonitoring */
    Hi_UVLO_thr_of_4_65_V,              /* VConnThresholdUVLO */
    5000,                               /* VBusSelect */
    0,                                  /* VbusVShiftHigh */
    0,                                  /* VbusVShiftLow */
    DISABLE,                            /* VBusRange */
    VBUS_vSafe0V_Thr_0_6V,              /* VBusThresholdVSafe0V */
    DISABLE,                            /* VddOVLO */
    DISABLE,                            /* VddUVLO */
    7,                                  /* VBusDischargeTimeTo0V */
    10,                                 /* VBusDischargeTimeToPDO */
    DISABLE,                            /* PowerAccessoryDetection */
    DISABLE                             /* PowerAccessoryTransition */
  },
#endif
};


extern USBPD_CAD_HandleTypeDef CAD_Handles[];


/* 
 * Lock parts based on CommLock var in the Ports handle 
 * 
 */
#define COMM_TO_DEFAULT   100
#define COMM_TO_NOWAIT    0
#define COMM_TO_INFINITE -1
/* 
  -1 for a infinite wait 
  0 no wait
*/
uint16_t LockWait = 0;
uint16_t LockCount = 0;
uint16_t LockRelease = 0;
static HAL_StatusTypeDef COMM_WAIT(uint8_t PortNum, int16_t Timeout)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  int16_t timeout = Timeout;
  LockWait++;
  while (1)
  {
    if (Ports[PortNum].CommLock == 0)
    {
      /* the resource is free */
      Ports[PortNum].CommLock = 1;
      ret = HAL_OK;
      LockCount++;
      break;
    }
    if (timeout == 0)
    {
      /* timeout */
      ret = HAL_TIMEOUT;
      break;
    }
    
    if (timeout > 0)
    {
      timeout--;
    }
  }
  return ret;
}

/* Function not used */
/* static HAL_StatusTypeDef COMM_LOCK(uint8_t PortNum)
{
  return COMM_WAIT(PortNum, 0);
}
*/

static HAL_StatusTypeDef COMM_RELEASE(uint8_t PortNum)
{
  LockRelease++;
  if (Ports[PortNum].CommLock == 0)
  {
    /* no change, the resource is already free */
    return HAL_ERROR;
  }

  /* release the resource */
  Ports[PortNum].CommLock = 0;
  return HAL_OK;
}
/* Inner function prototypes -----------------------------------------------*/

/* STUSB16xx init function*/
void HW_IF_STUSB1602_IO_Init(uint8_t PortNum);
#ifndef MB1303
void HW_IF_STUSB16xxEVAL_IO_Init(uint8_t PortNum);
#endif
void HW_IF_STUSB16xx_Reset(uint8_t PortNum);

void HW_IF_SPI_Init(uint8_t PortNum);
void HW_IF_STUSB16xx_I2C_Init(uint8_t PortNum);

void HW_IF_COUNTER_TIM_Init(uint8_t PortNum);


/* Port management functions */
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role);
void HW_IF_STUSB1602_Registers_Init(uint8_t PortNum);

void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc);
void HW_IF_DMA_Init(uint8_t PortNum);

/* IOs management functions */
#ifndef MB1303
void HW_IF_DischPath_SNK_Status(uint8_t PortNum, GPIO_PinState status);
void HW_IF_DischPath_SRC_Status(uint8_t PortNum, GPIO_PinState status);
void HW_IF_VBUS_EN_SRC_STM32_Status(uint8_t PortNum, GPIO_PinState status);
#endif
void HW_IF_RESET_Assert(uint8_t PortNum);
void HW_IF_RESET_Deassert(uint8_t PortNum);


/* SPI and NSS management functions */
void HW_IF_SPI_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode);
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState state);
void HW_IF_NSS_Rising_Interrupt (uint8_t PortNum ,FunctionalState state);
void HW_IF_NSS_Falling_Interrupt (uint8_t PortNum ,FunctionalState state);

uint8_t HW_IF_Check_VBus(uint8_t port);
void HW_IF_RX_Enable(uint8_t PortNum);
void HW_IF_RX_Disable(uint8_t PortNum);
HAL_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum);

void CAD_StateMachine(USBPD_CAD_HandleTypeDef* hcad, uint8_t port);
static HAL_StatusTypeDef HW_IF_HR_Start_ComplementaryActions(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);


/* Public functions ---------------------------------------------------------*/


/**
  * @brief  Initialize port harware interface
  * @param  PortNum: port index
  * @param  cbs:
  * @param  role:
  * @retval HAL_StatusTypeDef
  */ 


HAL_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role)
{
  nvm_read = 0;
  HAL_StatusTypeDef res = HAL_OK;
        
  /* Init of all IOs for the specified AFE port*/
  HW_IF_STUSB1602_IO_Init(PortNum);

#ifndef MB1303
  HW_IF_STUSB16xxEVAL_IO_Init(PortNum);
#endif  

  HW_IF_STUSB16xx_Reset(PortNum);

  /* following two instructions have to be managed in a better way*/
#if defined(MB1303) && USBPD_PORT_COUNT == 1
  /* Configuration of Port 1 in case of single port */
  HW_IF_STUSB1602_IO_Init(1);
  HW_IF_STUSB16xx_Reset(1);
#endif

  /* Init peripherals required by the specified port*/
  HW_IF_STUSB16xx_I2C_Init(PortNum);
  STUSB1602_Driver_Init(Ports[PortNum].hi2c);
  
 /* check init phase is completed on STUSB1602*/
  nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
  while (nvm_read != 2)
  {
    /*NVM not ready*/
    nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
  }
  /* Add check of chip ID*/ 
  Ports[PortNum].Device_cut = STUSB1602_DEVICE_CUT_Get(STUSB1602_I2C_Add(PortNum));
 
#if defined(MB1303) && USBPD_PORT_COUNT == 2
  {
  if (PortNum == 1) 
  { 
    while (Ports[0].Device_cut != Ports[1].Device_cut)
    {Ports[PortNum].Device_cut = STUSB1602_DEVICE_CUT_Get(STUSB1602_I2C_Add(PortNum));
   }
  }
  }
#endif
 
  
  /* DMA init */
  HW_IF_DMA_Init(PortNum);
  HW_IF_SPI_Init(PortNum);

  HW_IF_COUNTER_TIM_Init(PortNum);

  /* Set the power role of the port */
  HW_IF_Port_SetInitialRole(PortNum,role);

  /* Switch the port in RX mode */
  STUSB16xx_HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);  

  /* Alert interrupt init*/
  HW_IF_STUSB1602_Interrupt_CC_Detection(PortNum, ENABLE);
  
  /* Initialize State and callbacks */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  Ports[PortNum].cbs = cbs;
  Ports[PortNum].role = role;
  
  return res;
}


/**
  * @brief  Connect Rp resitors on the CC lines.
            This is done by set-up STUSB16xx in SRC mode
  * @param  PortNum: port index
  * @retval none 
  */
void USBPDM1_AssertRp(uint8_t PortNum)
{
  __NOP();
}


/**
  * @brief  Disconnect Rp resitors on the CC lines.
            However Rd resistors are connected by set-up STUSB16xx in SNK mode. 
  * @param  PortNum: port index
  * @retval none 
  */
void USBPDM1_DeAssertRp(uint8_t PortNum)
{
  __NOP();
}


/**
  * @brief  Request assert of Rd resitors on the CC lines.
  * @param  PortNum: port index
  * @retval none 
  */
void USBPDM1_AssertRd(uint8_t PortNum)
{
  __NOP();
}


/**
  * @brief  Disconnect Rd resitors on the CC lines.
            However Rd resistors are connected by set-up STUSB16xx in SRC mode. 
  * @param  PortNum: port index
  * @retval none 
  */
void USBPDM1_DeAssertRd(uint8_t PortNum)
{
  __NOP();
}



/**
  * @brief  Power role swap: Request assert of Rp resitors on the CC lines.
  * @param  PortNum: port index
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rp(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{ 
  HAL_StatusTypeDef status = HAL_OK;

  if (CurrentRole != USBPD_PORTPOWERROLE_SNK)
  {
    return HAL_ERROR;
  }

  /* i2c_pr_swap_rp_assert_req command */

  status = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_RP_ASSERT_REQ);
  return status;
  
}


/**
  * @brief  Power role swap: Request assert of Rd resitors on the CC lines.
  * @param  PortNum: port index
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rd(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  if (CurrentRole != USBPD_PORTPOWERROLE_SRC)
  {
    return HAL_ERROR;
  }

  /* i2c_pr_swap_rd_assert_req command */
  return (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_RD_ASSERT_REQ);
}


/**
  * @brief  Power role swap: VBUS OFF in sink role.
  * @param  PortNum: port index
  * @param  role: port power role
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef USBPD_HW_IF_PRS_Vbus_OFF(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  if (CurrentRole != USBPD_PORTPOWERROLE_SRC && CurrentRole != USBPD_PORTPOWERROLE_SNK)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef ret = HAL_ERROR;


  /* try to acquire the communication resource to avoid the conflict */
  ret = COMM_WAIT(PortNum, COMM_TO_DEFAULT);                           
  if (ret != HAL_OK)                                                   
  {                                                                    
    return ret;                                                  
  }                                                                    


  if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
  {
    /* work-around: The VBUS selected value is incremented to disable the monitoring */
    if (Ports[PortNum].Device_cut == Cut_1)
    {  
      uint16_t VBusValue = STUSB1602_VBUS_Select_Status_Get(STUSB1602_I2C_Add(PortNum));
      VBusValue = (VBusValue + 100); /* 100 heuristic value */
      ret = (HAL_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), VBusValue);
      if (ret != HAL_OK)
      {
        return ret;
      }
    }

    /* i2c_pr_swap_src_vbus_off_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_SRC_VBUS_OFF_REQ);

    /* Turn off the power */

    /* enabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Enable); 

    /* waiting for VSafe0V */
    HAL_Delay(150);

    /* disabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Disable);
  }
  else if (CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    /* i2c_pr_swap_snk_vbus_off_req command */
    /* i2c_pr_swap_src_vbus_off_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_SNK_VBUS_OFF_REQ);
  }

  COMM_RELEASE(PortNum);                        

  return ret;
}



/**
  * @brief  Power role swap: start of procedure.
  * @param  PortNum: port index
  * @param  role: port power role
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef USBPD_HW_IF_PRS_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode)
{
  if (CurrentRole != USBPD_PORTPOWERROLE_SRC && CurrentRole != USBPD_PORTPOWERROLE_SNK)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef ret = HAL_OK;

  /* try to acquire the communication resource to avoid the conflict */
  ret = COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != HAL_OK)
  {
    return ret;
  }

  /* Enable power role swap */
  ret = (HAL_StatusTypeDef)STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Enable);
  if (ret != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
  ret = (HAL_StatusTypeDef)STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_removed);    
  /* release the communication resource */
  /* added because PRS could be aborted before the end */ 
  COMM_RELEASE(PortNum);                        
  return ret == HAL_OK ? HAL_OK : HAL_ERROR;
}

/**
  * @brief  Power role swap: end of procedure.
  * @param  PortNum: port index
  * @param  role: port power role
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef USBPD_HW_IF_PRS_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  /* try to acquire the communication resource to avoid the conflict */
  /* added because PRS could be aborted before the end */
  ret = COMM_WAIT(PortNum, COMM_TO_DEFAULT);  
  if (ret != HAL_OK)                          
  {                                           
    return ret;                         
  }                                           

  if (CurrentRole == USBPD_PORTPOWERROLE_SRC) /* initially it was SNK */
  {
    ret = HAL_OK;
  }
  if (CurrentRole == USBPD_PORTPOWERROLE_SNK) /* initially it was SRC */
  {
    if (Ports[PortNum].Device_cut == Cut_1_A)
    {  
      ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_PS_RDY_REQ);
    }
    else
    {
      ret = (HAL_StatusTypeDef)STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_supp);
    }
  }
  HAL_Delay(15);

  /* Disable PRS functionality */

  /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
  ret = (HAL_StatusTypeDef)STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);

  /* release the communication resource */
  COMM_RELEASE(PortNum);

  return ret;
}

/**
* @brief  Hard Reset start procedure
* @param  PortNum: port index
* @param  mode: ACKNOWLEDGE or REQUEST
* @retval HAL_StatusTypeDef 
*/
uint32_t HRBalanceStart = 0x00;
uint32_t HRBalanceEnd = 0x00;

HAL_StatusTypeDef USBPD_HW_IF_HR_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode)
{
  HRBalanceStart++;
  HAL_StatusTypeDef ret = HAL_ERROR;

  /* try to acquire the communication resource to avoid the conflict */
  ret = COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != HAL_OK)
  {
    return ret;
  }

  if (Ports[PortNum].Device_cut == Cut_1)
  {  
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
  }

  /* set standard value of the vbus monitoring 5V */
  STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000);

  /* mask the cc line detection */
  ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), Mode == ACKNOWLEDGE ? PD_HARD_RESET_RECEIVED_REQ : PD_HARD_RESET_SEND_REQ);
  if (Mode == ACKNOWLEDGE)
  {
    uint16_t i;
    /* Waiting ~12us */
    for(i=0; i<200; i++)
    {
      __NOP();
    }
  }

  /* if ok call the complementary actions */
  if (ret == HAL_OK)
  {  
    ret = HW_IF_HR_Start_ComplementaryActions(PortNum, CurrentRole);
  }
  /* release the communication resource */
  COMM_RELEASE(PortNum);

  return ret;
}


/**
* @brief  Hard Reset complementary start actions
* @param  PortNum: port index
* @retval HAL_StatusTypeDef 
*/
static HAL_StatusTypeDef HW_IF_HR_Start_ComplementaryActions(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  HAL_StatusTypeDef ret = HAL_OK;
  STUSB1602_CC_DETECTION_STATUS_RegTypeDef STUSB1602_CC_DETECTION_STATUS_Value;

  /* Read the detection status register (0x0E) */
  STUSB1602_CC_DETECTION_STATUS_Value = STUSB1602_CC_Detection_Status_Get(STUSB1602_I2C_Add(PortNum));

  /* Check if VConn is provided */ 
  if (STUSB1602_CC_DETECTION_STATUS_Value.b.CC_VCONN_SUPPLY_STATE == VCONN_supplied_on_unused_CC_pin)   
  {
    /* i2c_hard_reset_turn_off_vconn_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_TURN_OFF_VCONN_REQ);    
  }

  /* When Device is source and data role is UFP it must be reset to DFP */
  if (
    (((Power_Role_TypeDef)STUSB1602_CC_DETECTION_STATUS_Value.b.CC_POWER_ROLE == Source) && 
    ((Data_Role_TypeDef)(STUSB1602_CC_DETECTION_STATUS_Value.b.CC_DATA_ROLE == UFP_data_mode)))
    )
  {
    /* i2c_hard_reset_port_change_2_dfp_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_PORT_CHANGE_2_DFP_REQ);    
  }


  /* When Device is sink and data role is DFP it must be reseted to UFP */
  if (
    (((Power_Role_TypeDef)STUSB1602_CC_DETECTION_STATUS_Value.b.CC_POWER_ROLE == Sink) && 
    ((Data_Role_TypeDef)(STUSB1602_CC_DETECTION_STATUS_Value.b.CC_DATA_ROLE == DFP_data_mode)))
    )
  {
    /* i2c_hard_reset_port_change_2_ufp_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_PORT_CHANGE_2_UFP_REQ);    
  }

  return ret;
}


/**
* @brief  Check if Vbus is below the safe voltage thereshold 
* @param  PortNum: port index
* @retval HAL_StatusTypeDef 
*/
HAL_StatusTypeDef USBPD_HW_IF_HR_CheckVbusVSafe0V(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
  {
    return HAL_OK;
  }
  if (CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    return STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_below_UVLO_threshold ? HAL_OK : HAL_BUSY;
  }
  return HAL_ERROR;
}


/**
* @brief  Hard Reset end procedure
* @param  PortNum: port index
* @retval HAL_StatusTypeDef 
*/
HAL_StatusTypeDef USBPD_HW_IF_HR_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  HRBalanceEnd++;
  HAL_StatusTypeDef ret = HAL_OK;
  /* try to acquire the communication resource to avoid the conflict */
  ret = COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != HAL_OK)
  {
    return ret;
  }

  /* i2c_hard_reset_complete_req command */
  ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_COMPLETE_REQ);    

  /* set standard value of the vbus monitoring 5V */
  ret = (HAL_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000);

  /* disable mask the cc on line detection */
  STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_UNMASKED);

  /* release the communication resource */
  COMM_RELEASE(PortNum);

  return ret;
}

/* add error recovery function */
HAL_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum)
{
  HAL_StatusTypeDef ret = HAL_OK;
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
  HAL_Delay(27); // need to be 25ms min
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);
  return ret;
}

/* Inner functions ---------------------------------------------------------*/


/**
  * @brief  SPI init function
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_SPI_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  SPI_HandleTypeDef*           phspi = &(Ports[PortNum].hspi);

  phspi->Instance =           SPI_Instance(PortNum);
  phspi->Init.Mode =           SPI_MODE_SLAVE;
  phspi->Init.Direction =  SPI_DIRECTION_2LINES;
  phspi->Init.DataSize =   SPI_DATASIZE_8BIT;
  phspi->Init.CLKPolarity =     SPI_POLARITY_HIGH;
  phspi->Init.CLKPhase =   SPI_PHASE_1EDGE;
  phspi->Init.NSS =     SPI_NSS_HARD_INPUT;
  phspi->Init.FirstBit =   SPI_FIRSTBIT_LSB;
  phspi->Init.TIMode =     SPI_TIMODE_DISABLE;
  phspi->Init.CRCCalculation =  SPI_CRCCALCULATION_DISABLE;
  phspi->Init.CRCPolynomial =   7;
  phspi->Init.CRCLength =   SPI_CRC_LENGTH_DATASIZE;
  phspi->Init.NSSPMode =   SPI_NSS_PULSE_DISABLE;

  HAL_SPI_Init(phspi);
}


/**
  * @brief  SPI is configured to sample data on rising edge on TX phase as well as
  *         SPI is configured to sample data on falling edge on RX phase
  * @param  PortNum: port index
  * @param  mode: SPI configured for transmission and reception
  * @retval None
  */ 
void HW_IF_SPI_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode)
{
  /* Stop the SPI DMA before changing SPI mode */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);

  /* Get the peripheral handler variable */
  SPI_HandleTypeDef*   phspi = &(Ports[PortNum].hspi);
  uint32_t             CR1Value;

  CR1Value = phspi->Instance->CR1;

  if (Ports[PortNum].Device_cut== Cut_1)
  {  
    phspi->Init.CLKPhase = (((CR1Value>>1)&1)^mode)&1;
    phspi->Instance->CR1 &= ~1;
    phspi->Instance->CR1 |= (((CR1Value>>1)&1)^mode)&1;
  }

  if (Ports[PortNum].Device_cut == Cut_1_A)
  {  
    phspi->Init.CLKPhase = (((CR1Value>>1)&1)^1)&1;
    phspi->Instance->CR1 &= ~1;
    phspi->Instance->CR1 |= (((CR1Value>>1)&1)^1)&1;
  }

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&Ports[PortNum].hspi);
}


/**
  * @brief  Enable the DMA clock.
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_DMA_Init(uint8_t PortNum)
{
  /* DMA controller clock enable */
  DMA_CLK_ENABLE(PortNum);

  /* NVIC configuration for DMA */
  HAL_NVIC_SetPriority(DMACHIRQ(PortNum), DMACHIRQ_PRIO(PortNum), 0);
}


/**
  * @brief  Initialize STUSB1602 GPIO Pins status after reset
  * @param  usbport: usb port index
  * @retval None
  */ 
void HW_IF_STUSB16xx_Reset(uint8_t PortNum)
{
  HW_IF_RESET_Assert(PortNum);

  uint16_t i;
  /* Waiting ~10us */
  for(i=0; i<120; i++)
  {
    __NOP();
  }

  HW_IF_RESET_Deassert(PortNum);


  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
#ifndef MB1303  
  HW_IF_DischPath_SNK_Status(PortNum, GPIO_PIN_RESET);
  HW_IF_DischPath_SRC_Status(PortNum, GPIO_PIN_RESET);
  HW_IF_VBUS_EN_SRC_STM32_Status(PortNum, GPIO_PIN_RESET);
#endif
}


/**
  * @brief  I2C init function
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_STUSB16xx_I2C_Init(uint8_t PortNum)
{
  I2C_HandleTypeDef*           phi2c = &(Ports[PortNum].hi2c);

  phi2c->Instance = I2C_INSTANCE(PortNum);
  phi2c->Init.Timing =  I2C_TIMING(PortNum); 
  phi2c->Init.OwnAddress1 = 0;
  phi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  phi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  phi2c->Init.OwnAddress2 = 0;
  phi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  phi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  phi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  HAL_I2C_Init(phi2c);

  HAL_I2CEx_ConfigAnalogFilter(phi2c, I2C_ANALOGFILTER_ENABLE);
}


/**
  * @brief  STUSB1602 IOs Initialization
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_STUSB1602_IO_Init(uint8_t PortNum)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pin : ALERT */
  GPIO_InitStruct.Pin = ALERT_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT_GPIO_PORT(PortNum), &GPIO_InitStruct);

  /* Configure GPIO pin : A_B_SIDE */
  GPIO_InitStruct.Pin = A_B_Side_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A_B_Side_GPIO_PORT(PortNum), &GPIO_InitStruct);  

  /* Configure GPIO pin : TX_EN */
  GPIO_InitStruct.Pin = TX_EN_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_PORT(PortNum), &GPIO_InitStruct);  

  /* Configure GPIO pins : RESET */
  GPIO_InitStruct.Pin = RESET_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_PORT(PortNum), &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ALERT_GPIO_IRQHANDLER(PortNum), ALERT_GPIO_IRQPRIORITY(PortNum), 0);
  HAL_NVIC_EnableIRQ(ALERT_GPIO_IRQHANDLER(PortNum));

  /* GPIO for test purposes */
  if (PortNum == 0)
  {
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}


#ifndef MB1303
/**
  * @brief  STUSB16xx_EVAL IOs Initialization
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_STUSB16xxEVAL_IO_Init(uint8_t PortNum)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pins : DischPath_SNK */
  GPIO_InitStruct.Pin = DISCH_PATH_SNK_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISCH_PATH_SNK_PORT(PortNum), &GPIO_InitStruct);

  /* Configure GPIO pins : DischPath_SRC */
  GPIO_InitStruct.Pin = DISCH_PATH_SRC_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISCH_PATH_SRC_PORT(PortNum), &GPIO_InitStruct);

  /* Configure GPIO pin : VBUS_EN_SRC_STM32 */
  GPIO_InitStruct.Pin = VBUS_EN_SRC_STM32_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VBUS_EN_SRC_STM32_PORT(PortNum), &GPIO_InitStruct);  


  /* GPIO for test purposes */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure unused GPIO pins as input analog */

  /* PA4 PA6 PA7 PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15; /* GPIO_PIN_4|*/
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB1 PB2 PB12 PB15 PB3 PB4 PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_15 
    |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
    |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PC15 PC0 PC1 PC4 PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4 
    |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  

  /* PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  

}


/**
  * @brief  Hardware interface DischPath_SNK control function
  * @param  PortNum: port index
  * @param  status: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_DischPath_SNK_Status(uint8_t PortNum, GPIO_PinState status)
{
  /* Sets/Reset the TX_EN pin associated to PortNum */
  HAL_GPIO_WritePin(DISCH_PATH_SNK_PORT(PortNum), DISCH_PATH_SNK_PIN(PortNum), status);
}


/**
  * @brief  Hardware interface DischPath_SRC control function
  * @param  PortNum: port index
  * @param  status: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_DischPath_SRC_Status(uint8_t PortNum, GPIO_PinState status)
{
  /* Sets/Reset the TX_EN pin associated to PortNum */
  HAL_GPIO_WritePin(DISCH_PATH_SRC_PORT(PortNum), DISCH_PATH_SRC_PIN(PortNum), status);
}


/**
  * @brief  Hardware interface VBUS_EN_SRC_STM32 control function
  * @param  PortNum: port index
  * @param  status: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_VBUS_EN_SRC_STM32_Status(uint8_t PortNum, GPIO_PinState status)
{
  /* Sets/Reset the TX_EN pin associated to PortNum */
  HAL_GPIO_WritePin(VBUS_EN_SRC_STM32_PORT(PortNum), VBUS_EN_SRC_STM32_PIN(PortNum), status);
}

#endif // ifndef MB1303

/**
  * @brief  Assert STUSB1602 software RESET
  * @param  PortNum: port index
  * @retval None
*/ 
void HW_IF_RESET_CTRL(uint8_t PortNum)
{
  uint16_t i;
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
  for(i=0; i<100; i++)
  {
    __NOP();
  }
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);
}


/**
  * @brief  Assert RESET
  * @param  PortNum: port index
  * @retval None
  */ 

void HW_IF_RESET_Assert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_SET);
}


/**
  * @brief  Desert RESET
  * @param  PortNum: port index
  * @retval None
  */ 
void HW_IF_RESET_Deassert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_RESET);
}


/**
  * @brief  STUSB1602 hardware RESET
  * @param  PortNum: port index
  * @retval None
*/ 
//void HW_IF_RESET_GPIO(uint8_t PortNum)
//{
//  uint16_t i;
//  HW_IF_RESET_Assert(PortNum);
//  for(i=0; i<1000; i++)
//  {__NOP();}  
//  HW_IF_RESET_Deassert(PortNum);    
//}


/**
  * @brief  Enable/Disable the CC Detection interrupt on ALERT pin
  * @param  PortNum: port index
  * @param  status: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_STUSB1602_Interrupt_CC_Detection(uint8_t PortNum, FunctionalState state)
{
  if (state == ENABLE)
  {
    STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_UNMASKED);
  }
  else
  {
    STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_MASKED);
  }
}


/**
  * @brief  RX Enable
  * @param  PortNum: port index
  * @retval None
  */
void HW_IF_RX_Enable(uint8_t PortNum)
{
  /* Set the port state to waiting */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  /* Enable the Rx interrupt if cbs are initialized; this mean that this is a PD capable port */
  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    __NOP();
  }
}


/**
  * @brief  RX disable
  * @param  PortNum: port index
  * @retval None
  */
void HW_IF_RX_Disable(uint8_t PortNum)
{
  /* The port is ready to transmit */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;

  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    /* Note (GN): To be completed with SPI management */ 
  }
}

/**
  * @brief  Configure STUSB1602 registers according to the initial role
  * @param  PortNum: port index
  * @param  role:
  * @retval None
  */ 
void HW_IF_STUSB1602_Registers_Init(uint8_t PortNum)
{
  uint8_t InitRegister = 0x00;

  /* Check the 0x0E register */
  if (STUSB1602_StartUp_Mode_Get(STUSB1602_I2C_Add(PortNum))!= Normal_Mode)
  {
    __NOP(); /* Error management should be implemented */
  } 

  /* Initialitazion of 0x18 register */
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_CAPABILITY_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0x20) | (((PortDeviceInit[PortNum].CCCurrentAdvertised) & 0x03) << 6) \
                                        | (((PortDeviceInit[PortNum].VConnDischarge) & 0x01) <<4) \
                                        | (((PortDeviceInit[PortNum].DataRoleSwap) & 0x01) << 3) \
                                        | (((PortDeviceInit[PortNum].PowerRoleSwap) & 0x01) << 2) \
                                        | (((PortDeviceInit[PortNum].VConnSwap) & 0x01) << 1) \
                                        | ((PortDeviceInit[PortNum].VConnSupply) & 0x01));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_CAPABILITY_CTRL_REG, 1);

  /* Initialitazion of 0x1E register*/
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_VCONN_SWITCH_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0xF0) | ((PortDeviceInit[PortNum].VConnIlim) & 0x0F));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_VCONN_SWITCH_CTRL_REG, 1);

  /* Initialitazion of 0x1F or 0x28 register */
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_MODE_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0xF8) | ((PortDeviceInit[PortNum].RolePower) & 0x07));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_MODE_CTRL_REG, 1);  

  /* Initialitazion of 0x20 register*/
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VCONN_MONITORING_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0x3F) | (((PortDeviceInit[PortNum].VConnMonitoring) & 0x01) << 7) \
                                        | (((PortDeviceInit[PortNum].VConnThresholdUVLO) & 0x01) << 6 ));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VCONN_MONITORING_CTRL_REG, 1);

  /* Initialitazion of 0x21 register */
  InitRegister = (PortDeviceInit[PortNum].VBusSelect);
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_SELECT_REG, 1);

  /* Initialitazion of 0x22 register */
  InitRegister = ((((PortDeviceInit[PortNum].VbusVShiftHigh) & 0x0F) << 4 ) \
                 | ((PortDeviceInit[PortNum].VbusVShiftLow) & 0x0F));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_RANGE_MONITORING_CTRL_REG, 1);

  /* Initialitazion of 0x24 register */
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_POWERED_ACCESSORY_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0xFC) | ((PortDeviceInit[PortNum].PowerAccessoryTransition & 0x01) << 1)  \
                                        | (PortDeviceInit[PortNum].PowerAccessoryDetection & 0x01));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_CC_POWERED_ACCESSORY_CTRL_REG, 1);

  /* Initialitazion of 0x25 register */
  InitRegister = ((((PortDeviceInit[PortNum].VBusDischargeTimeTo0V) & 0x0F) << 4 ) \
                 | ((PortDeviceInit[PortNum].VBusDischargeTimeToPDO) & 0x0F));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_DISCHARGE_TIME_CTRL_REG, 1);

  /* Initialitazion of 0x26 register */
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_DISCHARGE_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0x7F) | (((PortDeviceInit[PortNum].VBusDischarge) & 0x01) << 7));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_DISCHARGE_CTRL_REG, 1);  

  /* Initialitazion of 0x2E register */
  STUSB1602_ReadReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_MONITORING_CTRL_REG, 1); 
  InitRegister = ((InitRegister & 0x57) | (((PortDeviceInit[PortNum].VddOVLO) & 0x01) << 6) \
                                        | (((PortDeviceInit[PortNum].VBusRange) & 0x01) <<4 ) \
                                        | (((PortDeviceInit[PortNum].VBusThresholdVSafe0V) &0x01) << 1) \
                                        | ((PortDeviceInit[PortNum].VddUVLO) & 0x01));
  STUSB1602_WriteReg(&InitRegister, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_MONITORING_CTRL_REG, 1);  
}



/**
  * @brief  Configure STUSB16xx according to initial role
  * @param  PortNum: port index
  * @param  role:
  * @retval None
  */ 
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role)
{
  switch (role)
  {
    /* Sink */
  case USBPD_PORTPOWERROLE_SNK:

      /*0x18*/
      STUSB1602_Data_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Data_Role_Swap_Disable);
      STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Disable);
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
      STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
      if (Ports[PortNum].VConn == ENABLE)
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
      else
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);

      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), SNK_without_accessory_supp);  
      
      /*0x20*/
      STUSB1602_VCONN_Monitor_Status_Set(STUSB1602_I2C_Add(PortNum), Disable_UVLO_thr_detect_on_VCONN); 
      STUSB1602_VCONN_UVLO_Thresh_Status_Set(STUSB1602_I2C_Add(PortNum), Hi_UVLO_thr_of_4_65_V);
      
      /*0x21*/
      STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000); 
      
      /*0x22*/
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 10); 
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -10); 
      
      /*0x24*/
      STUSB1602_Pwr_Acc_Detect_Set(STUSB1602_I2C_Add(PortNum), Pwr_Acc_Detect_Disable); 
      
      /*0x25 def 84*7=588 */
      STUSB1602_VBUS_Discharge_Time_to_0V_Set(STUSB1602_I2C_Add(PortNum), 84*7);
      STUSB1602_VBUS_Discharge_Time_to_PDO_Set(STUSB1602_I2C_Add(PortNum), 200);
      
      /*0x2E*/
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
      STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V); /* default value is VBUS_vSafe0V_Thr_0_6V, VBUS_vSafe0V_Thr_1_8V */
      STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
    break;

    /* Source */
    case USBPD_PORTPOWERROLE_SRC:

      /*0x18*/  
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A); /* USB_C_Current_1_5_A */
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
      if (Ports[PortNum].VConn == ENABLE)
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
      else
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
      
      STUSB1602_Data_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Data_Role_Swap_Disable);    
      STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Disable);    

      /*0x1E*/
      STUSB1602_VCONN_Switch_Current_Limit_Set(STUSB1602_I2C_Add(PortNum), ILIM_350_ma);

      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), SRC_with_accessory_supp);  

      /*0x20*/
      STUSB1602_VCONN_Monitor_Status_Set(STUSB1602_I2C_Add(PortNum), Disable_UVLO_thr_detect_on_VCONN); 
      STUSB1602_VCONN_UVLO_Thresh_Status_Set(STUSB1602_I2C_Add(PortNum), Hi_UVLO_thr_of_4_65_V);

      /*0x21*/
      STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000); 

      /*0x22*/
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 10); 
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -10); 
      
      /*0x25*/
      STUSB1602_VBUS_Discharge_Time_to_0V_Set(STUSB1602_I2C_Add(PortNum), 84*7);
      STUSB1602_VBUS_Discharge_Time_to_PDO_Set(STUSB1602_I2C_Add(PortNum), 200);
   
      /* 0x026 enabling discharge */
      STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Enable); 
      
      /*0x2E*/
      STUSB1602_VDD_OVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_OVLO_Enable);
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
      STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V);
      STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);

    break;


    /* Dual Role */
    default:
      /*0x18*/  
#if defined(CONF_DEMO)
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_1_5_A);
#else
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A);
#endif
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
      STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);

      if (Ports[PortNum].VConn == ENABLE)
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
      else
          STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
      STUSB1602_Data_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Data_Role_Swap_Disable);    
      STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Disable);    

      /*0x1E*/
      STUSB1602_VCONN_Switch_Current_Limit_Set(STUSB1602_I2C_Add(PortNum), ILIM_350_ma);

      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_supp);  

      /*0x20*/
      STUSB1602_VCONN_Monitor_Status_Set(STUSB1602_I2C_Add(PortNum), Disable_UVLO_thr_detect_on_VCONN); 
      STUSB1602_VCONN_UVLO_Thresh_Status_Set(STUSB1602_I2C_Add(PortNum), Hi_UVLO_thr_of_4_65_V);

      /*0x21*/
      STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000); 


      /*0x22*/
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 10); 
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -10); 

      /*0x25*/
      STUSB1602_VBUS_Discharge_Time_to_0V_Set(STUSB1602_I2C_Add(PortNum), 84*7);
      STUSB1602_VBUS_Discharge_Time_to_PDO_Set(STUSB1602_I2C_Add(PortNum), 200);

      /*0x2E*/
      STUSB1602_VDD_OVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_OVLO_Enable);

#if defined(CONF_DEMO)
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
#else
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
#endif
      
      STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V); /* default value is VBUS_vSafe0V_Thr_0_6V, VBUS_vSafe0V_Thr_1_8V */
      STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
    break;
  } 
}


/**
  * @brief  Assign CC to Port Handle
  * @param  PortNum: port index
  * @param  cc: CC number
  * @retval None
  */ 
void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  /* Set the correct pin on the comparator*/
  Ports[PortNum].CCx = cc;
  Ports[PortNum].CCxChange = SET;
}


/**
* @brief  Check if VBus is present or not
* @param  port:  port
* @retval 1
*/
uint8_t HW_IF_Check_VBus(uint8_t PortNum)
{
  if(STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_above_UVLO_threshold)
    return 1;
  else
    return 0;
}


/**
  * @brief  Check if the bus is idle
  * @param  port: Specifies the port to check.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum)
{
  return (((Ports[PortNum].CCx == CCNONE) || ((HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET))) ? HAL_BUSY : HAL_OK);

}


/**
  * @brief  Enable/Diable Rising Falling Interrupts on NSS line
  * @param  port: port number
  * @param  state: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState state)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (state == ENABLE)
  {
    /* Configure NVIC */
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    /* Connect External Line to the GPIO*/
    LL_APB1_GRP2_EnableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
  }
  else
  {
    LL_APB1_GRP2_DisableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = DISABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_NONE;
    LL_EXTI_Init(&EXTI_InitStruct);
  }  
}


/**
  * @brief  Enable/Diable Falling Interrupts on NSS line
  * @param  port: port number
  * @param  state: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_NSS_Falling_Interrupt (uint8_t PortNum ,FunctionalState state)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (state == ENABLE)
  {
    /* Configure NVIC */
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    LL_APB1_GRP2_EnableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
  }
  else
  {
    LL_APB1_GRP2_DisableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 =  SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = DISABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_NONE;
    LL_EXTI_Init(&EXTI_InitStruct);
  }  
}


/**
  * @brief  Enable/Diable Rising Interrupts on NSS line
  * @param  port: port number
  * @param  state: ENABLE/DISBLE
  * @retval None
  */ 
void HW_IF_NSS_Rising_Interrupt (uint8_t PortNum ,FunctionalState state)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (state == ENABLE)
  {
    /* Configure NVIC */
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    LL_APB1_GRP2_EnableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);
  }
  else
  {
    LL_APB1_GRP2_DisableClock(SPI_NSS_LL_APB(PortNum));
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
    EXTI_InitStruct.LineCommand = DISABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_NONE;
    LL_EXTI_Init(&EXTI_InitStruct);
  }  
}


/**
  * @brief  COUNT_TIM init function
  * @param  port: port number
  * @retval None
  */ 
void HW_IF_COUNTER_TIM_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  TIM_HandleTypeDef* htimcountrx = &(Ports[PortNum].htimcountrx);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htimcountrx->Instance =                       RX_COUNTTIM(PortNum);
  htimcountrx->Init.Prescaler =                 ( HAL_RCC_GetHCLKFreq() / 1000000 ) - 1; // 1us Resolution
  htimcountrx->Init.CounterMode =               TIM_COUNTERMODE_UP;
  htimcountrx->Init.Period =                    DMA_TIME_ELAPSED;
  htimcountrx->Init.ClockDivision =             TIM_CLOCKDIVISION_DIV1;
  htimcountrx->Init.RepetitionCounter =         0;
  HAL_TIM_Base_Init(htimcountrx);

  HAL_TIM_OC_Init(htimcountrx);

  sBreakDeadTimeConfig.OffStateRunMode =        TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode =       TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel =              TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime =               0;
  sBreakDeadTimeConfig.BreakState =             TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity =          TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput =        TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(htimcountrx, &sBreakDeadTimeConfig);

  sConfigOC.OCMode =                            TIM_OCMODE_TIMING;
  sConfigOC.Pulse =                             DMA_TIME_COUNT_COMPARE;
  sConfigOC.OCPolarity =                        TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity =                       TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode =                        TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState =                       TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState =                      TIM_OCNIDLESTATE_RESET;
  HAL_TIM_OC_ConfigChannel(htimcountrx, &sConfigOC, RX_COUNTTIMCH(PortNum));

  __HAL_TIM_CLEAR_IT(htimcountrx, TIM_IT_UPDATE);
}



/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Init DMA for transmission
  * @param  PortNum: port index
  * @retval None
  */ 
void STUSB16xx_HW_IF_TX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);

  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].hspi.hdmatx = hdma_tx_spi;

  /* Peripheral DMA init*/
  hdma_tx_spi->Instance =                   TX_DMACH(PortNum);
  hdma_tx_spi->Init.Direction =             DMA_MEMORY_TO_PERIPH;
  hdma_tx_spi->Init.PeriphInc =             DMA_PINC_DISABLE;
  hdma_tx_spi->Init.MemInc =                DMA_MINC_ENABLE;
  hdma_tx_spi->Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
  hdma_tx_spi->Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
  hdma_tx_spi->Init.Mode =                  DMA_NORMAL;
  hdma_tx_spi->Init.Priority =              DMACHIRQ_PRIO(PortNum);
  HAL_DMA_Init(hdma_tx_spi);

  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));

  /* Enable IRQ DMA */
  HAL_NVIC_EnableIRQ(DMACHIRQ(PortNum));
}


/**
  * @brief  Init DMA for reception
  * @param  PortNum: port index
  * @retval None
  */ 
void STUSB16xx_HW_IF_RX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_rx_spi = &(Ports[PortNum].hdmarx);

  /* Peripheral DMA init*/
  hdma_rx_spi->Instance =                   RX_DMACH(PortNum);
  hdma_rx_spi->Init.Direction =             DMA_PERIPH_TO_MEMORY;
  hdma_rx_spi->Init.PeriphInc =             DMA_PINC_DISABLE;
  hdma_rx_spi->Init.MemInc =                DMA_MINC_ENABLE;
  hdma_rx_spi->Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
  hdma_rx_spi->Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
  hdma_rx_spi->Init.Mode =                  DMA_NORMAL;
  hdma_rx_spi->Init.Priority =              DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(hdma_rx_spi);

  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].hspi.hdmarx = hdma_rx_spi; 

  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmarx,(*hdma_rx_spi));

  /* Enable IRQ DMA */
  HAL_NVIC_EnableIRQ(DMACHIRQ(PortNum));

}


/**
  * @brief  Switch SPI DMA in normal mode
  * @param  PortNum: port index
  * @retval None
  */
void STUSB16xx_HW_IF_Set_DMA_Normal_Mode(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);

  hdma_tx_spi->Init.Mode =                  DMA_NORMAL;
  HAL_DMA_Init(hdma_tx_spi);

  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));
}


/**
  * @brief  Switch SPI DMA in circular mode
  * @param  PortNum: port index
  * @retval None
  */
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);

  hdma_tx_spi->Init.Mode =                  DMA_CIRCULAR;
  //  hdma_tx_spi->Init.MemInc =                DMA_MINC_DISABLE;
  HAL_DMA_Init(hdma_tx_spi);

  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMACHIRQ(PortNum), DMACHIRQ_PRIO(PortNum), 0);
}


/**
  * @brief  Switch SPI mode
  * @param  PortNum: port index
  * @param  mode: SPI configured for transmission and reception
  * @retval None
  */ 
void STUSB16xx_HW_IF_Switch_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode)
{

  /* Set the SPI CLK edge according to mode : always 1 for cut 1.3 , keep mode def for cut 1.2 */
  /* STUSB1602_CUT_1_2 */
  if (Ports[PortNum].Device_cut == Cut_1_A)
  {  
    HW_IF_SPI_Mode(PortNum,(STUSB1602_SPI_Mode_TypeDef)(1));  
  }
  else
  {  
    HW_IF_SPI_Mode(PortNum, mode);  
  }

  /* Enable/Disable RX NSS EXT Interrupt */
  HW_IF_NSS_RisingFalling_Interrupt (PortNum, mode == STUSB16xx_SPI_Mode_RX ? ENABLE : DISABLE);
}


/**
  * @brief  Hardware interface TX_EN control function
  * @param  PortNum: port index
  * @param  status: ENABLE/DISBLE
  * @retval None
  */ 
void STUSB16xx_HW_IF_TX_EN_Status(uint8_t PortNum, GPIO_PinState status)
{
  /* Sets/Reset the TX_EN pin associated to PortNum */
  HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum), status);
}
uint8_t nb_detach = 0;
/* 
the name is temporary, according to the old CAD architecture 
need to be changed
*/
void CAD_StateMachine(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum)
{
  assert_param(hcad != NULL);

  /* Alert management */
  STUSB16xx_PORT_HandleTypeDef * hhw_handle = &Ports[PortNum];
  if (hhw_handle->AlertEventCount > 0)
  {
    HAL_StatusTypeDef ret = HAL_ERROR;

    /* try to acquire the communication resource to avoid the conflict */
    ret = COMM_WAIT(PortNum, 0);          
    if (ret == HAL_OK)
    {
      if (STUSB16xx_HW_IF_Alert_Manager(PortNum) == HAL_OK)
      {
        /* The alert was correctly served */
        hhw_handle->AlertEventCount = 0;
      }
      COMM_RELEASE(PortNum);
    }
  }


  /* change of CAD state machine*/
  if ((hhw_handle->CCxChange == SET) && (hcad->state != USBPD_CAD_STATE_SWITCH_TO_SRC) && (hcad->state != USBPD_CAD_STATE_SWITCH_TO_SNK))
  {
    /* invoke the callback */
    if (( hcad->callback.USBPD_CAD_CallbackEvent) != NULL)
    {
      /* Forward event information to upper layer */ 
      hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state,  hcad->cc);
    }  

    /* reset the flag */
    hhw_handle->CCxChange = RESET;
  }

  if (Ports[PortNum].Device_cut == Cut_1_A)  
  {
    /* change of CAD state machine*/
    if (hcad->state == USBPD_CAD_STATE_DETACHED )
    {
      if (nb_detach == 0)
      {
        if ((STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum))) == VBUS_below_VSAFE0V_threshold)
        {
          nb_detach = 1;
        }
        else
        {   
          if  (STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == Unattached_SNK)
          {
            if ((STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum))) != VBUS_below_VSAFE0V_threshold)
            {
              HAL_Delay(100);
              if ((STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == AttachWait_SRC) && 
                  (STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum)) != VBUS_below_VSAFE0V_threshold))
              {     
                HW_IF_RESET_CTRL(PortNum);
              }
            }
          }//end if STUSB1602_TypeC_FSM_State_Get
        } // end else
      } 
    }
  }

  
  /* change of CAD state machine*/
  if (hcad->state == USBPD_CAD_STATE_ATTACHED )
  {
    nb_detach =0;
  }
}

/**
* @brief  It notifies that a new alert event occurred
* @param  PortNum: port index
* @retval None
*/ 
void STUSB16xx_HW_IF_Alert_Check(uint8_t PortNum)
{
  Ports[PortNum].AlertEventCount++;
}


/* debug only */
#include "stm32f0xx_hal_i2c.h"
extern I2C_HandleTypeDef STUSB16xx_I2CxHandle;
uint32_t I2C_LockCount = 0;


STUSB1602_ALERT_MONITORING_TypeDef            STUSB1602_AlertMonitoring_Value;
STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef  STUSB1602_Monitoring_Status_Trans_Value;
STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef    STUSB1602_HW_Fault_Status_Trans_Value;
/**
* @brief  It manages registers related to STUSB1602 ALERT interrupts
* @param  PortNum: port index
* @retval None
*/ 
HAL_StatusTypeDef STUSB16xx_HW_IF_Alert_Manager(uint8_t PortNum)
{

  uint8_t AlertAttempts;
  uint8_t AlertAccomplished;

  /* Delay of few us in order to be sure that registers are updated*/
  uint8_t i;
  for(i=0; i<200; i++)
  {
    __NOP();
  }  

  /* check alert bits to know what occured */
  AlertAccomplished = 0;
  AlertAttempts = 10;

  while(1)
  {
    /* Registers from 0x0B to 0x12 are read */
    STUSB1602_AlertMonitoring_Value = STUSB1602_Alert_Monitoring_Get(STUSB1602_I2C_Add(PortNum));

    /* Check ALERT due to a change occurred on CC */
    if ((STUSB1602_AlertMonitoring_Value.reg_0B.b.CC_DETECTION_STATUS_AL) && \
      (!STUSB1602_AlertMonitoring_Value.reg_0C.b.CC_DETECTION_STATUS_AL_MASK) && \
      (STUSB1602_AlertMonitoring_Value.reg_0D.b.ATTACH_STATE_TRANS))
    {
      if (STUSB1602_AlertMonitoring_Value.reg_0E.b.CC_ATTACH_STATE)     /* CC line is ATTACHED */
      {


        /* Check the CC attach mode */
        switch (STUSB1602_AlertMonitoring_Value.reg_0E.b.CC_ATTACH_MODE)
        {          
        case Sink_Attached:
          /* USBPD_CAD_STATE_ATTACHED */
          CAD_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SRC;
          break;

        case Source_Attached:
          /* USBPD_CAD_STATE_ATTACHED */
          CAD_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SNK;
          break;

        case Audio_Acc_Attached:
          /* USBPD_CAD_STATE_ACCESSORY */
          CAD_Handles[PortNum].state = USBPD_CAD_STATE_ACCESSORY;
          break;

        case Debug_Acc_Attached:
          /* USBPD_CAD_STATE_DEBUG */
          CAD_Handles[PortNum].state = USBPD_CAD_STATE_DEBUG;
          break;

        case Powered_Acc_Attached:
          /*  */
          break;

        default:
          /* USPPD_CAD_STATE_UNKNOWN */
          CAD_Handles[PortNum].state = USPPD_CAD_STATE_UNKNOW;
          break;
        }

        /* CAD handle is updated */
        CAD_Handles[PortNum].cc = CCXHANDLE(STUSB1602_AlertMonitoring_Value.reg_11.b.CC_ATTACHED);

        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum, CAD_Handles[PortNum].cc);

        /* RX mode is enabled */
        HW_IF_RX_Enable(PortNum);
      }
      else  /* CC line is DETACHED */
      {
        /* CAD handle is updated */
        CAD_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
        CAD_Handles[PortNum].cc = CCNONE;

        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum,CAD_Handles[PortNum].cc);

        /* TX mode is enabled */
        HW_IF_RX_Disable(PortNum);
      }

      /* Exit from the alert check procedure */
      AlertAccomplished = 1;             
    }


    /* Check if a MONITORING STATUS ALERT detected */
    if ((STUSB1602_AlertMonitoring_Value.reg_0B.b.MONITORING_STATUS_AL) && \
      (!STUSB1602_AlertMonitoring_Value.reg_0C.b.MONITORING_STATUS_AL_MASK)) 
    {
      /* Check changes occurred in MONITORING STATUS register and restore the ALERT pin */
      STUSB1602_Monitoring_Status_Trans_Value = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));

      /* Exit from the alert check procedure */
      AlertAccomplished = 1;

      /* If the alert due to a monitoring event is unmasked appropriate actions have to be added here */
      __NOP();
    }


    /* Check if a HW FAULT STATUS ALERT detected */
    if ((STUSB1602_AlertMonitoring_Value.reg_0B.b.HW_FAULT_STATUS_AL) && \
      (!STUSB1602_AlertMonitoring_Value.reg_0C.b.HW_FAULT_STATUS_AL_MASK)) 
    {
      /* Check changes occurred in HW FAULT STATUS register and restore the ALERT pin */
      STUSB1602_HW_Fault_Status_Trans_Value = STUSB1602_Hard_Fault_Trans_Status_Get(STUSB1602_I2C_Add(PortNum));

      /* Exit from the alert check procedure */
      AlertAccomplished = 1;

      /* If the alert due to a hardware fault event is unmasked appropriate actions have to be added here*/
      __NOP();
    }


    AlertAttempts--;
    if (AlertAttempts == 0 || AlertAccomplished)
    {
      break;
    }

    /* wait a short time before to try again */
    uint8_t i;
    for(i=0; i<10; i++)
    {
      __NOP();
    } 

    return HAL_OK;
  }

  /* reset alert signal */
  uint8_t count = 10;
  uint8_t cleared_alert_flag = 0;
  while (1)
  {
    cleared_alert_flag = (HAL_GPIO_ReadPin(ALERT_GPIO_PORT(PortNum), ALERT_GPIO_PIN(PortNum)) != GPIO_PIN_RESET);
    /* check if the alert pin is reset */
    if (cleared_alert_flag || count == 0) 
    {
      break;
    }
    if (STUSB16xx_I2CxHandle.Lock != HAL_LOCKED)
    {
      uint16_t i;
      /* Restore the CC_DETECTION_STATUS_TRANS register*/
      STUSB1602_Attach_State_Trans_Get(STUSB1602_I2C_Add(PortNum));

      /* Restore the MONITORING_STATUS_TRANS register */
      STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));

      /* Restore the HARD_FAULT_TRANS register */
      STUSB1602_Hard_Fault_Trans_Status_Get(STUSB1602_I2C_Add(PortNum));

      /* Decrement the counter to implement a TO */
      count--;

      /* wait a bit time */
      for(i=0; i<1000; i++)
      {
        __NOP();
      } 
    }
  }

  /* test last time if the flag was reset correctly */
  cleared_alert_flag |= (HAL_GPIO_ReadPin(ALERT_GPIO_PORT(PortNum), ALERT_GPIO_PIN(PortNum)) != GPIO_PIN_RESET);
  if (!cleared_alert_flag) {
    /* I2C broken */
    /* error led feedback and stop */
    USBPD_BSP_LED_Off(GREEN_USER_LED);
    USBPD_BSP_LED_Off(LED_PORT0_CC);
    USBPD_BSP_LED_Off(LED_PORT0_VBUS);
    USBPD_BSP_LED_Off(LED_PORT0_ROLE);
    USBPD_BSP_LED_Off(LED_PORT1_CC);
    USBPD_BSP_LED_Off(LED_PORT1_VBUS);
    USBPD_BSP_LED_Off(LED_PORT1_ROLE);
    while(1)
    {
      uint32_t i;
      USBPD_BSP_LED_Toggle(PortNum == 0 ? LED_PORT0_CC : LED_PORT1_CC);
      for(i=0; i<400000; i++) __NOP();
    }
  }
  return HAL_OK;
} 


/**
  * @brief  Send packet to STUSB1602
  * @param  port: port number
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef STUSB16xx_HW_IF_Send_Packet(uint8_t PortNum, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  /* Check if the bus is idle */
#ifdef DBG_STUSB1602
  ret = HAL_OK;
#else
  ret = HW_IF_check_bus_idle(PortNum);
#endif
  if (ret == HAL_OK && 1 /* tx ready */)
  {
    /* Set the state to busy*/
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_TX;

    /* Set the SPI in TX mode */
    STUSB16xx_HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);

    HAL_SPI_DMAStop(&Ports[PortNum].hspi);
    HAL_DMA_Abort(&Ports[PortNum].hdmarx);
    __HAL_DMA_CLEAR_FLAG(hdma, 0x0FFFFFFF);

    /* Send TX Buffer by SPI DMA */
    HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, pData, Size);  

    /* Set TX_EN GPIO */
    STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
  }
  else
  {
    __NOP();
  }
  return ret;
}


/**
  * @brief  VConn swap management
  * @param  PortNum: port index
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef STUSB16xx_HW_IF_VConnSwap(uint8_t PortNum)
{
  uint32_t STUSB16xx_ACK_timeout = 0xFFFF;
  HAL_StatusTypeDef ret = HAL_ERROR;

  if (Ports[PortNum].VConn == DISABLE)
  {
    /* i2c_vconn_swap_turn_on_vconn_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_ON_VCONN_REQ);    

    /* Check if ACK is got or a timeout is expired */
    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum))!=PD_VCONN_SWAP_Turn_On_VCONN_Ack) && STUSB16xx_ACK_timeout)
    {
      STUSB16xx_ACK_timeout--;
    }
  }
  else
  {
    /* i2c_vconn_swap_turn_on_vconn_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_OFF_VCONN_REQ);    

    /* Check if ACK is got or a timeout is expired */
    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum))!=PD_VCONN_SWAP_Turn_Off_VCONN_Ack) && STUSB16xx_ACK_timeout)
    {
      STUSB16xx_ACK_timeout--;
    } 
  }
  if (STUSB16xx_ACK_timeout == 0)
    ret = HAL_ERROR;

  return ret;
}


/**
  * @brief  Data Role swap management
  * @param  PortNum: port index
  * @retval HAL_StatusTypeDef 
  */
HAL_StatusTypeDef STUSB16xx_HW_IF_DataRoleSwap(uint8_t PortNum)
{
  uint32_t STUSB16xx_ACK_timeout = 0xFFFF;
  HAL_StatusTypeDef ret = HAL_ERROR;

  if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_DFP)
  {
    /* i2c_dr_swap_port_change_2_ufp_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ);    

    /* Check if ACK is got or a timeout is happen */
    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum)) != PD_DR_SWAP_Port_Change_2_UFP_Ack) && STUSB16xx_ACK_timeout)
    {
      STUSB16xx_ACK_timeout--;
    }
  }
  else if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_UFP)
  {
    /* i2c_dr_swap_port_change_2_dfp_req command */
    ret = (HAL_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ);    

    /* Check if ACK is got or a timeout is happen */
    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum)) != PD_DR_SWAP_Port_Change_2_DFP_Ack) && STUSB16xx_ACK_timeout)
    {
      STUSB16xx_ACK_timeout--;
    }
  }
  else
    ret = HAL_ERROR;

  if (STUSB16xx_ACK_timeout == 0)
    ret = HAL_ERROR;

  return ret;
}


/**
  * @brief  Set the VBus monitoring voltage as well as voltage range
  * @param  port: port number
  * @param  VBus: the value (expressed in mV) to set the internal reference DAC
  * @param  Hset: the VShift_High value >= 5% (expressed in %) to set the high threshold value  
  * @param  Lset: the VShift_Low value <=5% (expressed in %) to set the low threshold value  
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef STUSB16xx_HW_IF_Set_VBus_Monitoring(uint8_t PortNum, uint16_t VBus, uint8_t Hset, uint8_t Lset)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  /* check for parameters*/
  if (Hset > 15) Hset = 15;
  if (Lset > 15) Lset = 15;

  /* Sets the VBUS_SELECT DAC reference for VBUS sensing (bit7:0 0x21) */
  ret = (HAL_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), VBus);

  /* Sets the VBUS_VShift_High (bit7:4 0x22) */
  ret = (HAL_StatusTypeDef)STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), Hset);

  /* Sets the VBUS_VShift_Low (bit3:0 0x22) */
  ret = (HAL_StatusTypeDef)STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), Lset);

  return ret;
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
