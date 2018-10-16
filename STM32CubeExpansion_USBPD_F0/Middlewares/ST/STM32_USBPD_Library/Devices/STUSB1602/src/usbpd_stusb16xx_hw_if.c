/**
  ******************************************************************************
  * @file    usbpd_stusb16xx_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface cad functions.
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


/* Includes ------------------------------------------------------------------*/
#include "usbpd_hw_if.h"
#include "string.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_timersserver.h"
#include "usbpd_porthandle.h"
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"
#include "p-nucleo-usb002.h"

/* Includes for LL libraries */
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "cmsis_os.h"

/* Uncomment to show as warning the selected cut */
//#define SHOW_CUT_WARNING


/** @addtogroup STM32_USBPD_LIBRARY
 * @{
 */

/** @addtogroup USBPD_DEVICE
 * @{
 */

/** @addtogroup USBPD_DEVICE_HW_IF
 * @{
 */

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF
 * @{
 */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


#define COMM_TO_DEFAULT   100   /*!< Communication timeout value */
#define COMM_TO_NOWAIT    0     /*!< Communication timeout is zero */
#define COMM_TO_INFINITE -1     /*!< Communication timeout is infinite */

/**
  * @def STUSB16xx_STATUS_AL_MASK
  * @brief   The following definition is used to mask event interrupt and to prevent the assertion of the alert bit in the ALERT_STATUS register
  * @details STUSB16xx_CC_DETECTION_STATUS_AL_MASK masks CC detection alerts
  * @details STUSB16xx_MONITORING_STATUS_AL_MASK maks monitoring alerts
  * @details STUSB16xx_FAULT_STATUS_AL_MASK masks fault alerts
*/
#define STUSB16xx_STATUS_AL_MASK = STUSB16xx_MONITORING_STATUS_AL_MASK | STUSB16xx_FAULT_STATUS_AL_MASK;


/* Private macro -------------------------------------------------------------*/

/**
  @def CCXHANDLE(__CC__)
  @brief It matches CC handle and CC line number
*/
#define CCXHANDLE(__CC__) ((CCxPin_TypeDef) (__CC__ +1))

/* Private variables ---------------------------------------------------------*/
uint8_t nvm_read = 0;                   /*!< Variable used to check if NVM has been loaded correctly */

extern uint8_t          RXBuffer0[];    /* Buffer for raw data received on port 0 */
extern uint32_t         TXBuffer0[];    /* Buffer for data to be transmitted on port 0 */
#if (USBPD_PORT_COUNT == 2)
extern uint8_t          RXBuffer1[];    /* Buffer for raw data received on port 1 */
extern uint32_t         TXBuffer1[];    /* Buffer for data to be transmitted on port 1 */
#endif

extern CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT]; /*!CAD state handle Structure */
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];

/**
  *  \warning   Position of USBPD_HW_IF_ErrorRecovery function has to be reviewed. If it is public it have be moved outside this file
  */
USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum);


/**
  * @brief  Handle for the ports inside @ref USBPD_DEVICE_HW_IF
  */
#ifdef __VVAR
STUSB16xx_PORT_HandleTypeDef Ports[2] =
#else
STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT] =
#endif
{
  { 0,                          /* USBPD PORT number 0 */
    (uint8_t*)TXBuffer0,        /* Pointer to Tx Buffer of port 0 */
    0,                          /* Tx Transfer size on port 0 */
    (uint8_t*)RXBuffer0,        /* Pointer to Raw Rx transfer Buffer of port 0 */
    NULL,                       /* Pointer to 5bdecoded data of port 0 */
    PHY_MAX_RAW_SIZE,           /* Rx Transfer size on port 0 */
    CCNONE,                     /* CC pin used for communication on port 0 */
    RESET,                      /* CC event change flag of port 0 */
    HAL_UNLOCKED,               /* Locking object of port 0 */
    HAL_USBPD_PORT_STATE_RESET, /* Communication state of port 0 */
    0,                          /* Error code of port 0 */
#if defined(_SRC)
    USBPD_PORTPOWERROLE_SRC,    /* As default the Port 0 plays the Provider power role */
#else
    USBPD_PORTPOWERROLE_SNK,    /* As default the Port 0 plays the Consumer power role */
#endif
    0,                          /* Index for monitoring BIST Msg bits on port 0 */
    ENABLE,                     /* VConn status flag of port 0 */
#if defined(_SRC)
    USBPD_PORTDATAROLE_DFP,     /* As default the Port 0 plays the DFP data role */
#else
    USBPD_PORTDATAROLE_UFP,     /* As default the Port 0 plays the UFP data role */
#endif
    0,                          /* Tx spare bits on port 0 */
  },
#if (USBPD_PORT_COUNT == 2)
   { 1,                         /* USBPD PORT number 1 */
    (uint8_t*)TXBuffer1,        /* Pointer to Tx Buffer of port 1 */
    0,                          /* Tx Transfer size on port 1 */
    (uint8_t*)RXBuffer1,        /* Pointer to Raw Rx transfer Buffer of port 1 */
    NULL,                       /* Pointer to 5bdecoded data of port 1 */
    PHY_MAX_RAW_SIZE,           /* Rx Transfer size on port 1 */
    CCNONE,                     /* CC pin used for communication on port 1 */
    RESET,                      /* CC event change flag of port 1 */
    HAL_UNLOCKED,               /* Locking object of port 1 */
    HAL_USBPD_PORT_STATE_RESET, /* Communication state of port 1 */
    0,                          /* Error code of port 1 */
#if defined(_SRC)
    USBPD_PORTPOWERROLE_SRC,    /* As default the Port 1 plays the Provider power role */
#else
    USBPD_PORTPOWERROLE_SNK,    /* As default the Port 1 plays the Consumer power role */
#endif
    0,                          /* Index for monitoring BIST Msg bits on port 1 */
    ENABLE,                     /* VConn status flag of port 1 */
#if defined(_SRC)
    USBPD_PORTDATAROLE_DFP,     /* As default the Port 1 plays the DFP data role */
#else
    USBPD_PORTDATAROLE_UFP,     /* As default the Port 1 plays the UFP data role */
#endif
    0,                          /* Tx spare bits on port 1 */
   },
#endif
};


/**
  * @brief  USBPD Initialization structure inside @ref USBPD_DEVICE_HW_IF
  */
USBPD_Init_TypeDef PortDeviceInit[USBPD_PORT_COUNT] =
{
  { 0,                                                          /* USBPD_PORT number 0 */
    /* Enum mismatch: PortDeviceInit.RolePower = SNK_without_accessory_supp => 2 => USBPD_PORTPOWERROLE_DRP_SNK, TO BE CHECK! (added cast to avoid warning) */
#if defined(_DRP)
    (USBPD_PortPowerRole_TypeDef)DRP_w_accessory_supp,          /* As default the Port 0 device plays the DRP power role without accessory support */
    USBPD_PORTDATAROLE_DFP,                                     /* As default the Port 0 plays the DFP data role */
#elif defined(_SRC)
    (USBPD_PortPowerRole_TypeDef)SRC_with_accessory_supp,       /* As default the Port 0 device plays the Provider power role with accessory support */
    USBPD_PORTDATAROLE_DFP,                                     /* As default the Port 0 plays the DFP data role */
#else /* _SNK */
    (USBPD_PortPowerRole_TypeDef)SNK_without_accessory_supp,    /* As default the Port 0 device plays the Consumer power role without accessory support */
    USBPD_PORTDATAROLE_UFP,                                     /* As default the Port 0 plays the UFP data role */
#endif
#if defined(_VDM)
    ENABLE,                                                     /* Vendor messages */
#else
    DISABLE,                                                    /* Vendor messages */
#endif
    DISABLE,                                                    /* Ping message */
    DISABLE,                                                    /* Extended messages */
    0,                                                          /* Number of source capabilities requests before hard reset */
    USB_C_Current_Default,                                      /* Current capability advertised */
    DISABLE,                                                    /* Data role swap capability */
    DISABLE,                                                    /* Power role swap capability */
    DISABLE,                                                    /* VCONN swap capability */
    DISABLE,                                                    /* VCONN supply capability on CC pin */
    DISABLE,                                                    /* VCONN discharge on CC pin */
    ENABLE,                                                     /* VBus discharge capability */
    VConn_Ilim_350mA,                                           /* Default current limit supplying VCONN on the CC pins */
    DISABLE,                                                    /* UVLO threshold detection on VCONN pin */
    Hi_UVLO_thr_of_4_65_V,                                      /* UVLO threshold on VCONN pin */
    50,                                                         /* Number of 100mV steps related to targeted VBUS voltage used for VBUS range monitoring */
    5,                                                          /* Shift coefficient used for computing the high threshold value (5% + assigned value) of the monitoring voltage range */
    5,                                                          /* Shift coefficient used for computing the low threshold value (-5% - assigned value) of the monitoring voltage range */
    DISABLE,                                                    /* VBUS voltage range detection */
    VBUS_vSafe0V_Thr_0_6V,                                      /* VBus vSafe0V threshold */
    DISABLE,                                                    /* OVLO threshold detection on Vdd voltage */
    DISABLE,                                                    /* UVLO threshold detection on Vdd voltage */
    7,                                                          /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to 0 V: 84 ms*TDISPARAM (840ms is default discharge time) */
    10,                                                         /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to PDO: 20 ms*TDISPARAM (200 ms is default discharge time) */
    DISABLE,                                                    /* Powered accessory detection */
    DISABLE                                                     /* Powered accessory transition from Powered.Accessory state to Try.SNK */
  },
#if (USBPD_PORT_COUNT == 2)
   { 1,                                                         /* USBPD_PORT number 1 */
#if defined(_DRP)
    (USBPD_PortPowerRole_TypeDef)DRP_w_accessory_supp,          /* As default the Port 1 device plays the DRP power role without accessory support */
    USBPD_PORTDATAROLE_DFP,                                     /* As default the Port 1 plays the DFP data role */
#elif defined(_SRC)
    (USBPD_PortPowerRole_TypeDef)SRC_with_accessory_supp,       /* As default the Port 1 device plays the Provider power role with accessory support */
    USBPD_PORTDATAROLE_DFP,                                     /* As default the Port 1 plays the DFP data role */
#else /* _SNK */
    (USBPD_PortPowerRole_TypeDef)SNK_without_accessory_supp,    /* As default the Port 1 device plays the Consumer power role without accessory support */
    USBPD_PORTDATAROLE_UFP,                                     /* As default the Port 1 plays the UFP data role */
#endif
#if defined(_VDM)
    ENABLE,                                                     /* Vendor messages */
#else
    DISABLE,                                                    /* Vendor messages */
#endif
    DISABLE,                                                    /* Ping message */
    DISABLE,                                                    /* Extended messages */
    0,                                                          /* Number of source capabilities requests before hard reset */
    USB_C_Current_Default,                                      /* Current capability advertised */
    DISABLE,                                                    /* Data role swap capability */
    DISABLE,                                                    /* Power role swap capability */
    DISABLE,                                                    /* VCONN swap capability */
    DISABLE,                                                    /* VCONN supply capability on CC pin */
    DISABLE,                                                    /* VCONN discharge on CC pin */
    DISABLE,                                                    /* VBus discharge capability */
    VConn_Ilim_350mA,                                           /* Default current limit supplying VCONN on the CC pins */
    DISABLE,                                                    /* UVLO threshold detection on VCONN pin */
    Hi_UVLO_thr_of_4_65_V,                                      /* UVLO threshold on VCONN pin */
    50,                                                         /* Number of 100mV steps related to targeted VBUS voltage used for VBUS range monitoring */
    5,                                                          /* Shift coefficient used for computing the high threshold value (5% + assigned value) of the monitoring voltage range */
    5,                                                          /* Shift coefficient used for computing the low threshold value (-5% - assigned value) of the monitoring voltage range */
    DISABLE,                                                    /* VBUS voltage range detection */
    VBUS_vSafe0V_Thr_0_6V,                                      /* VBus vSafe0V threshold */
    DISABLE,                                                    /* OVLO threshold detection on Vdd voltage */
    DISABLE,                                                    /* UVLO threshold detection on Vdd voltage */
    7,                                                          /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to 0 V: 84 ms*TDISPARAM (840ms is default discharge time) */
    10,                                                         /* Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to PDO: 20 ms*TDISPARAM (200 ms is default discharge time) */
    DISABLE,                                                    /* Powered accessory detection */
    DISABLE                                                     /* Powered accessory transition from Powered.Accessory state to Try.SNK */
  },
#endif
};


/* Inner function prototypes -----------------------------------------------*/

/* COMM functions*/
static USBPD_StatusTypeDef HW_IF_COMM_WAIT(uint8_t PortNum, int16_t Timeout);
static USBPD_StatusTypeDef HW_IF_COMM_RELEASE(uint8_t PortNum);

/* STUSB16xx init function*/
void HW_IF_STUSB1602_IO_Init(uint8_t PortNum);

/* Initialization of STUSB1602 GPIO pins status after reset */
void HW_IF_STUSB16xx_Reset(uint8_t PortNum);

/* Initialization of SPI peripheral */
void HW_IF_SPI_Init(uint8_t PortNum);

/* Initialization of I2C peripheral */
void HW_IF_STUSB16xx_I2C_Init(uint8_t PortNum);

/* Initialization of counter peripheral */
void HW_IF_COUNTER_TIM_Init(uint8_t PortNum);

/* Port management functions */
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role);
void HW_IF_STUSB1602_Registers_Init(uint8_t PortNum);
void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc);
void HW_IF_DMA_Init(uint8_t PortNum);

/* IOs management functions */
void HW_IF_RESET_Assert(uint8_t PortNum);
void HW_IF_RESET_Deassert(uint8_t PortNum);


/* SPI and NSS management functions */
void HW_IF_SPI_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode);
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState status);
void HW_IF_NSS_Rising_Interrupt (uint8_t PortNum ,FunctionalState status);
void HW_IF_NSS_Falling_Interrupt (uint8_t PortNum ,FunctionalState status);

USBPD_FunctionalState HW_IF_Check_VBus(uint8_t port);
void HW_IF_RX_Enable(uint8_t PortNum);
void HW_IF_RX_Disable(uint8_t PortNum);
USBPD_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum);

static USBPD_StatusTypeDef HW_IF_HR_Start_ComplementaryActions(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);


/* Public functions ----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Public_Functions USBPD DEVICE STUSB16XX HW IF Public functions
  * @details Public functions can be used at stack level
  * @{
  */

/**
  * @brief  It Initializes port harware interface
  * @param  PortNum The port index
  * @param  cbs The callbacks exposed by the HW_IF to the PHY
  * @param  role The port power role
  * @retval USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role)
{
  USBPD_StatusTypeDef res = USBPD_OK;
  nvm_read = 0;

  /* Init of all IOs for the specified AFE port*/
  HW_IF_STUSB1602_IO_Init(PortNum);

  HW_IF_STUSB16xx_Reset(PortNum);

  /* Init peripherals required by the specified port*/
  HW_IF_STUSB16xx_I2C_Init(PortNum);
  STUSB1602_Driver_Init(PortNum, Ports[PortNum].hi2c);
#ifdef __VVAR
  HW_IF_STUSB16xx_I2C_Init(1);
  STUSB1602_Driver_Init(1, Ports[1].hi2c);
#endif

 /* check init phase is completed on STUSB1602*/
  nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
  while (nvm_read != 2)
  {
    /*NVM not ready*/
    nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
  }
  /* Add check of chip ID*/
  Ports[PortNum].Device_cut = STUSB1602_DEVICE_CUT_Get(STUSB1602_I2C_Add(PortNum));

#if USBPD_PORT_COUNT == 2
  if (PortNum == 1)
  {
    while (Ports[0].Device_cut != Ports[1].Device_cut)
      {
        Ports[PortNum].Device_cut = STUSB1602_DEVICE_CUT_Get(STUSB1602_I2C_Add(PortNum));
   }
  }
#endif

  /* SPI and DMA init */
  HW_IF_DMA_Init(PortNum);
  HW_IF_SPI_Init(PortNum);

  /* Timer init */
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
  * @brief   It connects Rp resitor on the CC lines
  * @details This function is left empty because Rp is asserted by STUSB16xx when it acts the Provider power role
  * @param   PortNum The port index
  * @retval none
  */
void USBPDM1_AssertRp(uint8_t PortNum)
{
}


/**
  * @brief   It disconnects Rp resitor on the CC lines
  * @details This function is left empty because Rp is denied by STUSB16xx when it acts the Provider power role
  * @param   PortNum The port index
  * @retval none
  */
void USBPDM1_DeAssertRp(uint8_t PortNum)
{
}


/**
  * @brief   It connects Rd resitor on the CC lines
  * @details This function is left empty because Rd is asserted by STUSB16xx when it acts the Consumer power role
  * @param   PortNum The port index
  * @retval none
  */
void USBPDM1_AssertRd(uint8_t PortNum)
{
}


/**
  * @brief   It disconnects Rd resitor on the CC lines
  * @details This function is left empty because Rd is denied by STUSB16xx when it acts the Consumer power role
  * @param   PortNum The port index
  * @retval none
  */
void USBPDM1_DeAssertRd(uint8_t PortNum)
{
}

/**
  * @brief   Power role swap: Rp resitor on the CC line
  * @details It requests Rp assertion on CC line as step of power role swap USB PD transaction
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @retval  USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rp(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if ((USBPD_FALSE == _handle->settings->CAD_RoleToggle) && (CurrentRole != USBPD_PORTPOWERROLE_SNK))
  {
    return USBPD_ERROR;
  }

  /* i2c_pr_swap_rp_assert_req command */
  return (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_RP_ASSERT_REQ);
}


/**
  * @brief   Power role swap: Rd resitor on the CC line
  * @details It requests Rd assertion on CC line as step of power role swap USB PD transaction
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @retval  USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rd(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if ((USBPD_FALSE == _handle->settings->CAD_RoleToggle) && (CurrentRole != USBPD_PORTPOWERROLE_SRC))
  {
    return USBPD_ERROR;
  }

  /* i2c_pr_swap_rd_assert_req command */
  return (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_RD_ASSERT_REQ);
}


/**
  * @brief   Power role swap: VBUS OFF in sink role
  * @details It requests VBUS OFF as step of power role swap USB PD transaction
  * @param   PortNum The port index
  * @param   CurrentRole  The port power role
  * @retval  USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Vbus_OFF(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if (USBPD_FALSE == _handle->settings->CAD_RoleToggle)
  {
    return USBPD_ERROR;
  }

  USBPD_StatusTypeDef ret = USBPD_ERROR;
  /* try to acquire the communication resource to avoid the conflict */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
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
    ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), VBusValue);
    if (ret != USBPD_OK)
    {
      return ret;
    }
    }

    /* i2c_pr_swap_src_vbus_off_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_SRC_VBUS_OFF_REQ);

    /* Turn off the power */

    /* enabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Enable);

    /* waiting for VSafe0V */
    while (STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum)) != VBUS_below_VSAFE0V_threshold)
    {
      for(uint32_t i = 0; i < 0xFFFF; i++) __NOP();
      osDelay(10);
    }

    /* disabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Disable);
  }
  else if (CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    /* i2c_pr_swap_snk_vbus_off_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_SNK_VBUS_OFF_REQ);
  }
  HW_IF_COMM_RELEASE(PortNum);
  return ret;
}



/**
  * @brief   Power role swap: start of procedure
  * @details It notifies to the STUSB16xx device that PRS transaction is starting
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @param   Mode           Two allowed values: ACKNOWLEDGE or REQUEST
  * @retval  USBPD status
  */
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if (USBPD_FALSE == _handle->settings->CAD_RoleToggle)
  {
    return USBPD_ERROR;
  }

  USBPD_StatusTypeDef ret = USBPD_OK;

  /* try to acquire the communication resource to avoid the conflict */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
  {
    return ret;
  }

  /* Enable power role swap */
  ret = (USBPD_StatusTypeDef)STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Enable);
  if (ret != USBPD_OK)
  {
    return USBPD_ERROR;
  }

  /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
  ret = (USBPD_StatusTypeDef)STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
  /* release the communication resource */
  /* added because PRS could be aborted before the end */
   HW_IF_COMM_RELEASE(PortNum);
  return ret == USBPD_OK ? USBPD_OK : USBPD_ERROR;
}



/**
  * @brief  Power role swap: end of procedure
  * @details It notifies to the STUSB16xx device that PRS transaction is ending
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @retval USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef USBPD_HW_IF_PRS_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if (USBPD_FALSE == _handle->settings->CAD_RoleToggle)
  {
    ret = USBPD_OK;
  }

  /* try to acquire the communication resource to avoid the conflict */
  /* added because PRS could be aborted before the end */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
  {
    return ret;
  }

 if (CurrentRole == USBPD_PORTPOWERROLE_SRC) /* initially it was SNK */
  {
    ret = USBPD_OK;
  }

  if (CurrentRole == USBPD_PORTPOWERROLE_SNK) /* initially it was SRC */
  {
    if (Ports[PortNum].Device_cut == Cut_1_A)
    {
      ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_PR_SWAP_PS_RDY_REQ);
    }
    else
    {
      ret = (USBPD_StatusTypeDef)STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_supp);
    }
  }
  osDelay(15);

  /* Disable PRS functionality */
//  ret = (USBPD_StatusTypeDef)STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Disable);

  /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
  ret = (USBPD_StatusTypeDef)STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);

  /* release the communication resource */
  HW_IF_COMM_RELEASE(PortNum);

  return ret;
}


/**
  * @brief   Hard Reset: start of procedure
  * @details It notifies to the STUSB16xx device that PRS transaction is starting
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @param   Mode           Two allowed values: ACKNOWLEDGE or REQUEST
  * @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_HR_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;

  /* try to acquire the communication resource to avoid the conflict */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
  {
    return ret;
  }

  if (Ports[PortNum].Device_cut == Cut_1)
  {
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
  }

  /* set standard value of the vbus monitoring 5V */
  STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000);

  /* pd_hard_reset_received_req or pd_hard_reset_send_req command */
  ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), Mode == ACKNOWLEDGE ? PD_HARD_RESET_RECEIVED_REQ : PD_HARD_RESET_SEND_REQ);
  if (Mode == ACKNOWLEDGE)
  {
    /* Waiting ~12us */
    for(uint16_t i=0;i<200;i++)
    {
      __NOP();
    }
  }

  /* if ok call the complementary actions */
  if (ret == USBPD_OK)
  {
    ret = HW_IF_HR_Start_ComplementaryActions(PortNum, CurrentRole);
  }
  /* release the communication resource */
  HW_IF_COMM_RELEASE(PortNum);

  return ret;
}


/**
  * @brief   Hard Reset: start of complementary actions
  * @param   PortNum The port index
  * @param   CurrentRole    The port power role
  * @retval  USBPD_StatusTypeDef
*/
static USBPD_StatusTypeDef HW_IF_HR_Start_ComplementaryActions(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  USBPD_StatusTypeDef ret = USBPD_OK;
  STUSB1602_CC_DETECTION_STATUS_RegTypeDef STUSB1602_CC_DETECTION_STATUS_Value;

  /* Read the detection status register (0x0E) */
  STUSB1602_CC_DETECTION_STATUS_Value = STUSB1602_CC_Detection_Status_Get(STUSB1602_I2C_Add(PortNum));

  /* Check if VConn is provided */
  if (STUSB1602_CC_DETECTION_STATUS_Value.b.CC_VCONN_SUPPLY_STATE == VCONN_supplied_on_unused_CC_pin)
  {
    /* i2c_hard_reset_turn_off_vconn_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_TURN_OFF_VCONN_REQ);
  }

  /* When Device is source and data role is UFP it must be reset to DFP */
  if (
      (((Power_Role_TypeDef)STUSB1602_CC_DETECTION_STATUS_Value.b.CC_POWER_ROLE == Source) &&
       ((Data_Role_TypeDef)(STUSB1602_CC_DETECTION_STATUS_Value.b.CC_DATA_ROLE == UFP_data_mode)))
        )
  {
    /* i2c_hard_reset_port_change_2_dfp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_PORT_CHANGE_2_DFP_REQ);
  }

  /* When Device is sink and data role is DFP it must be reseted to UFP */
  if (
      (((Power_Role_TypeDef)STUSB1602_CC_DETECTION_STATUS_Value.b.CC_POWER_ROLE == Sink) &&
       ((Data_Role_TypeDef)(STUSB1602_CC_DETECTION_STATUS_Value.b.CC_DATA_ROLE == DFP_data_mode)))
        )
  {
    /* i2c_hard_reset_port_change_2_ufp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_PORT_CHANGE_2_UFP_REQ);
  }

  return ret;
}
 /* update Ports according to data role */
USBPD_StatusTypeDef USBPD_HW_IF_DataRole(uint8_t PortNum)
{
  Ports[PortNum].DataRole = (USBPD_PortDataRole_TypeDef)STUSB1602_Data_Role_Get(PortNum);

  return USBPD_ERROR;
}
USBPD_StatusTypeDef USBPD_HW_IF_ResetDataRole(uint8_t PortNum)
{
#if defined(_SRC)
  Ports[PortNum].DataRole = USBPD_PORTDATAROLE_DFP;
#else
  Ports[PortNum].DataRole = USBPD_PORTDATAROLE_UFP;
#endif
  return USBPD_ERROR;
} 

/**
  * @brief   It checks if Vbus is below the safe voltage threshold
  * @param   PortNum The port index
  * @param   CurrentRole  The current port power role
  * @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_HR_CheckVbusVSafe0V(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
  {
    return USBPD_OK;
  }
  if (CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    return STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_below_UVLO_threshold ? USBPD_OK : USBPD_BUSY;
  }
  return USBPD_ERROR;
}


/**
  * @brief   Hard Reset: end of procedure
  * @details It notifies to the STUSB16xx device that PRS transaction is ending
  * @param   PortNum The port index
  * @param   CurrentRole The current port power role
  * @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_HR_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  USBPD_StatusTypeDef ret = USBPD_OK;

  /* try to acquire the communication resource to avoid the conflict */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
  {
    return ret;
  }

  /* i2c_hard_reset_complete_req command */
  ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Command(STUSB1602_I2C_Add(PortNum), PD_HARD_RESET_COMPLETE_REQ);    
  
  /* set standard value of the vbus monitoring 5V */
  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000);
  
  /* disable mask the cc on line detection */
  STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_UNMASKED);

  /* release the communication resource */
  HW_IF_COMM_RELEASE(PortNum);

  return ret;
}


/**
  * @brief   Error recovery function
  * @param   PortNum The port index
  * @retval  USBPD status 
*/
USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_OK;
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
////////  /* CC line is DETACHED */  // BAZIN not interrupt generated to update the CAD status 
////////  /* CAD handle is updated */
////////  CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
////////  CAD_HW_Handles[PortNum].cc = CCNONE;
////////
////////  /* Port handle is updated */
////////  HW_IF_Port_Set_CC(PortNum,CAD_HW_Handles[PortNum].cc);
////////
////////  /* TX mode is enabled */
////////  HW_IF_RX_Disable(PortNum);

  HAL_Delay(27); // need to be 25ms min
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);

  return ret;
}

/**
 * @}
 */

/* Inner functions -----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Inner_Functions USBPD DEVICE STUSB16XX HW IF Inner functions
  * @details Inner functions can be used at file level
  * @{
  */

/**
  * @brief   It waits for the communication sequence with the device is completed
  * @param   PortNum The port index
  * @param   Timeout The timeout time
  * @retval  USBPD status 
*/
static USBPD_StatusTypeDef HW_IF_COMM_WAIT(uint8_t PortNum, int16_t Timeout)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  int16_t timeout = Timeout;
  while (1)
  {
    if (Ports[PortNum].CommLock == 0)
    {
      /* the resource is free */
      Ports[PortNum].CommLock = 1;
      ret = USBPD_OK;
      break;
    }
    if (timeout == 0)
    {
      /* timeout */
      ret = USBPD_TIMEOUT;
      break;
    }

    if (timeout > 0)
    {
      timeout--;
    }
  }
  return ret;
}


/**
  * @brief   It releases the communication resources
  * @param   PortNum The port index
  * @retval  USBPD status 
*/
static USBPD_StatusTypeDef HW_IF_COMM_RELEASE(uint8_t PortNum)
{
  if (Ports[PortNum].CommLock == 0)
  {
    /* no change, the resource is already free */
    return USBPD_ERROR;
  }

  /* release the resource */
  Ports[PortNum].CommLock = 0;
  return USBPD_OK;
}


/**
  * @brief  SPI init function
  * @param  PortNum The port index
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
  * @brief  SPI configuration according to trasmission or reception phase 
  * @param  PortNum The port index
  * @param  mode Two allowed values: STUSB16xx_SPI_Mode_TX or STUSB16xx_SPI_Mode_RX
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
  uint32_t              CR1Value;
  
  CR1Value = phspi->Instance->CR1;
  
  /* If cut 1.2 is used, SPI is configured to sample data
     on rising edge on TX phase as well as on falling edge on RX phase */
  if (Ports[PortNum].Device_cut== Cut_1)
  {  
    phspi->Init.CLKPhase = (((CR1Value>>1)&1)^mode)&1;
    phspi->Instance->CR1 &= ~1;
    phspi->Instance->CR1 |= (((CR1Value>>1)&1)^mode)&1;
  }

  /* If cut 1.3 is used, SPI is configured to sample data
     on falling edge on TX phase as well as RX phase */
  if (Ports[PortNum].Device_cut == Cut_1_A)
  {  
    phspi->Init.CLKPhase = (((CR1Value>>1)&1)^1)&1;
    phspi->Instance->CR1 &= ~1;
    phspi->Instance->CR1 |= (((CR1Value>>1)&1)^1)&1;

    /* SPI NSS software or hardware according to the mode value */
    phspi->Instance->CR1 &= ~(1<<SPI_CR1_SSM_Pos);
    phspi->Instance->CR1 |= ((((~mode) & 1)<<SPI_CR1_SSM_Pos) & SPI_CR1_SSM);
  }
  
  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&Ports[PortNum].hspi);
}


/**
  * @brief  DMA init function
  * @param  PortNum The port index
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
  * @brief  I2C init function
  * @param  PortNum The port index
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
  * @brief  Initialization of STUSB1602 GPIO pins status after reset
  * @param  PortNum The port index
  * @retval None
  */ 
void HW_IF_STUSB16xx_Reset(uint8_t PortNum)
{
  HW_IF_RESET_Assert(PortNum);

  /* Waiting ~10us */
  for(uint16_t i=0;i<120;i++) 
    __NOP();

  HW_IF_RESET_Deassert(PortNum);

  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
}


/**
  * @brief  Configuration of STUSB1602 GPIO pins
  * @param  PortNum The port index
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
 
  /* pin for ADC*/
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
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


/**
  * @brief  STUSB16xx software reset
  * @param  PortNum The port index
  * @retval None
*/ 
void HW_IF_RESET_CTRL(uint8_t PortNum)
{
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
  for(uint16_t i=0; i<100; i++)
  {
    __NOP();
  }  
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);
}


/**
  * @brief  Assert STUSB16xx hardware reset
  * @param  PortNum The port index
  * @retval None
  */ 
void HW_IF_RESET_Assert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_SET);
}


/**
  * @brief  Desert STUSB16xx hardware reset
  * @param  PortNum The port index
  * @retval None
  */ 
void HW_IF_RESET_Deassert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_RESET);
}


/**
  * @brief  It enables or disables the notification of the CC Detection interrupt on ALERT pin
  * @param  PortNum The port index
  * @param  status Two allowed values: GPIO_PIN_RESET, GPIO_PIN_SET
  * @retval None
  */ 
void HW_IF_STUSB1602_Interrupt_CC_Detection(uint8_t PortNum, FunctionalState status)
{
  if (status == ENABLE)
  {
    STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_UNMASKED);
  }
  else
  {
    STUSB1602_CC_Detect_Alrt_Int_Mask_Set(STUSB1602_I2C_Add(PortNum), CC_Detect_Int_MASKED);
  }
}


/**
  * @brief  It updates the Ports handle stating the beginning of RX phase
  * @param  PortNum The port index
  * @retval None
  */
void HW_IF_RX_Enable(uint8_t PortNum)
{
  /* Set the port state to waiting */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;

  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    __NOP();
  }
}


/**
  * @brief  It updates the Ports handle stating the ending of RX phase
  * @param  PortNum The port index
  * @retval None
  */
void HW_IF_RX_Disable(uint8_t PortNum)
{
  /* The port is ready to transmit */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  
  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    __NOP();
  }
}


/**
  * @brief  Initialization of the STUSB16xx registers according to the initial role
  * @param  PortNum The port index
  * @retval None
  */ 
void HW_IF_STUSB1602_Registers_Init(uint8_t PortNum)
{
  uint8_t InitRegister = 0x00;

  /* Check the 0x0E register */
  if (STUSB1602_StartUp_Mode_Get(STUSB1602_I2C_Add(PortNum))!= Normal_Mode)
  {__NOP(); /* Error management should be implemented */ } 
  
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
  * @brief  Initialization of the STUSB16xx registers according to the initial role
  * @param  PortNum The port index
  * @param  role
  * @retval None
  */ 
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  
  if (USBPD_TRUE == _handle->settings->CAD_RoleToggle)
  {
    /* Dual Role */
    /*0x18*/  
#if defined(CONF_DEMO)
    STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_1_5_A);
#else
    STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A);
#endif
    STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
#ifdef _APPLI_VCONN_SUPPORT
    STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);    
#else
    STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif
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
    STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 20); 
    STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -20); 

    /*0x25*/
    STUSB1602_VBUS_Discharge_Time_to_0V_Set(STUSB1602_I2C_Add(PortNum), 84*7);
    STUSB1602_VBUS_Discharge_Time_to_PDO_Set(STUSB1602_I2C_Add(PortNum), 200);

    /*0x2E*/
    STUSB1602_VDD_OVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_OVLO_Enable);

#if defined(CONF_DEMO)
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
#else
 //   STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
#endif

    STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V); /* default value is VBUS_vSafe0V_Thr_0_6V, VBUS_vSafe0V_Thr_1_8V */
    STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
  }
  else
  {
    switch (role)
    {
      /* Consumer power role */
    case USBPD_PORTPOWERROLE_SNK:

      /*0x18*/
      STUSB1602_Data_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Data_Role_Swap_Disable);
      STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Disable);
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
      STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
#ifdef _APPLI_VCONN_SUPPORT
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
#else
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif

      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), SNK_without_accessory_supp);  

      /*0x20*/
      STUSB1602_VCONN_Monitor_Status_Set(STUSB1602_I2C_Add(PortNum), Disable_UVLO_thr_detect_on_VCONN); 
      STUSB1602_VCONN_UVLO_Thresh_Status_Set(STUSB1602_I2C_Add(PortNum), Hi_UVLO_thr_of_4_65_V);

      /*0x21*/
      STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000); 

      /*0x22*/
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 20); 
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -20); 

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

      /* Provider power role */
    case USBPD_PORTPOWERROLE_SRC:

      /*0x18*/  
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A); /* USB_C_Current_3_A */
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
#ifdef _APPLI_VCONN_SUPPORT
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
#else
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif

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
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 20);
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -20); 

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

#ifdef __VVAR
  STUSB1602_WriteRegSingle(0x91, 0x28, 0x2E);
  STUSB1602_WriteRegSingle(0x91, 0x29, 0x2E);
#endif
    break;


    /* Dual role power */
    default:

      /*0x18*/  
#if defined(CONF_DEMO)
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_1_5_A);
#else
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A);
#endif
      STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
      STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);

#ifdef _APPLI_VCONN_SUPPORT
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
#else
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif  
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
      STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 20); 
      STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -20); 

      /*0x25*/
      STUSB1602_VBUS_Discharge_Time_to_0V_Set(STUSB1602_I2C_Add(PortNum), 84*7);
      STUSB1602_VBUS_Discharge_Time_to_PDO_Set(STUSB1602_I2C_Add(PortNum), 200);

      /*0x2E*/
      STUSB1602_VDD_OVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_OVLO_Enable);
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);

      STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V); /* default value is VBUS_vSafe0V_Thr_0_6V, VBUS_vSafe0V_Thr_1_8V */
      STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
    break;
    }
  }
}


/**
  * @brief  It assigns CC line number to Port Handle
  * @param  PortNum The port index
  * @param  cc CC line number
  * @retval None
  */
void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  Ports[PortNum].CCx = cc;
  Ports[PortNum].CCxChange = SET;
}


/**
  * @brief  It checks if VBus is present or not
  * @param  PortNum The port index
  * @retval USBPD_FunctionalState
*/
USBPD_FunctionalState HW_IF_Check_VBus(uint8_t PortNum)
{
  return ((STUSB1602_VBUS_Presence_Get(STUSB1602_I2C_Add(PortNum)) == VBUS_above_UVLO_threshold) ? USBPD_ENABLE : USBPD_DISABLE);
}


/**
  * @brief  It checks if the bus is idle
  * @param  PortNum The port index
  * @retval USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum)
{
  return (((Ports[PortNum].CCx == CCNONE) || ((HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET))) ? USBPD_BUSY : USBPD_OK);
}


/**
  * @brief  It enables or disables the rising and falling interrupt on NSS line
  * @param  PortNum The port index
  * @param  status Two allowed values: ENABLE or DISABLE
  * @retval None
  */ 
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState status)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (status == ENABLE)
  {
    /* NVIC configuration*/
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    /* External Line initialization */
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
    /* External Line deinitialization */
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
  * @brief  It enables or disables the falling interrupt on NSS line
  * @param  PortNum The port index
  * @param  status Two allowed values: ENABLE or DISABLE
  * @retval None
  */ 
void HW_IF_NSS_Falling_Interrupt (uint8_t PortNum ,FunctionalState status)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (status == ENABLE)
  {
    /* Configure NVIC */
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    /* External Line initialization */
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
    /* External Line deinitialization */
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
  * @brief  It enables or disables the rising interrupt on NSS line
  * @param  PortNum The port index
  * @param  status Two allowed values: ENABLE or DISABLE
  * @retval None
  */ 
void HW_IF_NSS_Rising_Interrupt (uint8_t PortNum ,FunctionalState status)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  if (status == ENABLE)
  {
    /* Configure NVIC */
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));

    /* External Line initialization */
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
    /* External Line deinitialization */
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
  * @brief  Initialization of counter
  * @param  PortNum The port index
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

/**
 * @}
 */


/* Private functions ---------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Private_Functions USBPD DEVICE STUSB16XX HW IF Private functions
  * @details Private functions can be used at hardware interface level
  * @{
  */

/**
  * @brief  Initialization of DMA for transmission
  * @param  PortNum The port index
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
  * @brief  Initialization DMA for reception
  * @param  PortNum The port index
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
  * @brief  It switches SPI DMA in normal mode
  * @param  PortNum The port index
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
  * @brief  It switches SPI DMA in circular mode
  * @param  PortNum The port index
  * @retval None
  */
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);
  
  hdma_tx_spi->Init.Mode =                  DMA_CIRCULAR;
  HAL_DMA_Init(hdma_tx_spi);
  
  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));
  
  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMACHIRQ(PortNum), DMACHIRQ_PRIO(PortNum), 0);
}


/**
  * @brief  It changes SPI configuartion according to trasmission or reception mode requirements
  * @param  PortNum The port index
  * @param  mode Two allowed values:  STUSB16xx_SPI_Mode_TX or STUSB16xx_SPI_Mode_RX
  * @retval None
  */ 
void STUSB16xx_HW_IF_Switch_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode)
{

  /* Set the data sampling edge according to mode */
  HW_IF_SPI_Mode(PortNum, mode);  
  
  /* Enable/Disable RX NSS EXT Interrupt */
  HW_IF_NSS_RisingFalling_Interrupt (PortNum, mode == STUSB16xx_SPI_Mode_RX ? ENABLE : DISABLE);
}


/**
  * @brief  TX_EN GPIO control function
  * @details It sets or reset the TX_EN pin
  * @param  PortNum The port index
  * @param  status Two allowed values: GPIO_PIN_SET or GPIO_PIN_RESET
  * @retval None
  */ 
void STUSB16xx_HW_IF_TX_EN_Status(uint8_t PortNum, GPIO_PinState status)
{
  HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum), status);
}

/**
  * @brief  CAD state machine
  * @details It returns details on CAD event referred to the CC line
  * @param  PortNum The port index
  * @param  Event Pointer to USBPD_CAD_EVENT 
  * @param  CCXX Pointer to CCxPin_TypeDef
  * @retval uint32_t
*/
uint32_t CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX)
{
  static USBPD_CAD_EVENT previous_event[USBPD_PORT_COUNT] = 
         {
           USBPD_CAD_EVENT_NONE
#if USBPD_PORT_COUNT == 2
           , USBPD_CAD_EVENT_NONE
#endif /*USBPD_PORT_COUNT == 2*/
         };
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];

  *Event = USBPD_CAD_EVENT_NONE;

  /* Alert management */
  STUSB16xx_PORT_HandleTypeDef * hhw_handle = &Ports[PortNum];
  if (hhw_handle->AlertEventCount > 0)
  {
    USBPD_StatusTypeDef ret = USBPD_ERROR;

    /* try to acquire the communication resource to avoid the conflict */
    ret = HW_IF_COMM_WAIT(PortNum, 0);
    if (ret == USBPD_OK)
    {
      if (STUSB16xx_HW_IF_Alert_Manager(PortNum) == USBPD_OK)
      {
        /* The alert was correctly served */
        hhw_handle->AlertEventCount = 0;
      }
      HW_IF_COMM_RELEASE(PortNum);
    }
  }

  /*Check CAD STATE*/
  switch(_handle->state)
  {
  case USBPD_CAD_STATE_DETACHED :
    {
    if (Ports[PortNum].NbDetach == 0)
    {
      if ((STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum))) == VBUS_below_VSAFE0V_threshold)
      {
        Ports[PortNum].NbDetach = 1;
      }
      else
      {
        /* cover the case where stusb1602 becomes unattached to SRC and VBUS at strange level kept by other device */
        if  (STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == Unattached_SNK)
        {
          if ((STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum))) != VBUS_below_VSAFE0V_threshold)
          {
            if ((STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == AttachWait_SRC) && 
                (STUSB1602_VBUS_VSAFE0V_Get(STUSB1602_I2C_Add(PortNum)) != VBUS_below_VSAFE0V_threshold))
            {
              HW_IF_RESET_CTRL(PortNum);
            }
          }
        }/* end if STUSB1602_TypeC_FSM_State_Get */
      } /* end else */
    } 
    *Event = USBPD_CAD_EVENT_DETACHED;
    break;
    } /* end case */

  case USBPD_CAD_STATE_RESET :
    *Event = USBPD_CAD_EVENT_DETACHED;
    break;
  case USBPD_CAD_STATE_EMC :
  case USBPD_CAD_STATE_ACCESSORY :
  case USBPD_CAD_STATE_ATTEMC :
  case USBPD_CAD_STATE_ATTACHED :
    {
      Ports[PortNum].NbDetach =0;
    }

    break;
    
  case USBPD_CAD_STATE_SWITCH_TO_SRC :
    _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
    _handle->state = USBPD_CAD_STATE_ATTACHED;
    *Event = USBPD_CAD_EVENT_ATTACHED;
    Ports[PortNum].NbDetach =0;
    break;
    
  case USBPD_CAD_STATE_SWITCH_TO_SNK :
    _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
    _handle->state = USBPD_CAD_STATE_ATTACHED;
    *Event = USBPD_CAD_EVENT_ATTACHED;
    Ports[PortNum].NbDetach =0;
    break;
    
  default:
    /* nothing to do */
    Ports[PortNum].NbDetach =0;
    break;
  }
  
  /* change of CAD state machine*/
  if ((hhw_handle->CCxChange == SET) && (_handle->state != USBPD_CAD_STATE_SWITCH_TO_SRC) && (_handle->state != USBPD_CAD_STATE_SWITCH_TO_SNK))
  {
    *CCXX  = _handle->cc;
    
    /* reset the flag */
    hhw_handle->CCxChange = RESET;
  }
  /* Report Detach event only if no already reported */
  if ((*Event != USBPD_CAD_EVENT_DETACHED) || (previous_event[PortNum] != USBPD_CAD_EVENT_DETACHED))
  {
    /* report Event */
    previous_event[PortNum] = *Event;
  }
  else
  {
    /* report USBPD_CAD_EVENT_NONE */
    *Event = USBPD_CAD_EVENT_NONE;
  }

  return 2;
}


/**
  * @brief  It notifies that a new alert event occurred
  * @param  PortNum The port index
  * @retval None
  */ 
void STUSB16xx_HW_IF_Alert_Check(uint8_t PortNum)
{
  Ports[PortNum].AlertEventCount++;
}


/* debug only */
#include "stm32f0xx_hal_i2c.h"
extern I2C_HandleTypeDef STUSB16xx_I2CxHandle;
#ifdef __VVAR
extern I2C_HandleTypeDef STUSB16xx_I2CxHandle_P1;
#endif
uint32_t I2C_LockCount = 0;



/**
  * @brief  It manages registers related to STUSB1602 ALERT interrupts
  * @param  PortNum The port index
  * @retval USBPD_StatusTypeDef
  */ 
USBPD_StatusTypeDef STUSB16xx_HW_IF_Alert_Manager(uint8_t PortNum)
{
  STUSB1602_ALERT_MONITORING_TypeDef            STUSB1602_AlertMonitoring_Value;
  STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef  STUSB1602_Monitoring_Status_Trans_Value;
  STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef    STUSB1602_HW_Fault_Status_Trans_Value;
  uint8_t AlertAttempts;
  uint8_t AlertAccomplished;
  uint32_t i;

  /* Delay of few us in order to be sure that registers are updated*/
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
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SRC;
          break;

        case Source_Attached:
          /* USBPD_CAD_STATE_ATTACHED */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SNK;
          break;

        case Audio_Acc_Attached:
          /* USBPD_CAD_STATE_ACCESSORY */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_ACCESSORY;
          break;

        case Debug_Acc_Attached:
          /* USBPD_CAD_STATE_DEBUG */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DEBUG;
          break;

        case Powered_Acc_Attached:
          /*  */
          break;

        default:
          /* USPPD_CAD_STATE_UNKNOWN */
          CAD_HW_Handles[PortNum].state = USPPD_CAD_STATE_UNKNOW;
          break;
        }

        /* CAD handle is updated */
        CAD_HW_Handles[PortNum].cc = CCXHANDLE(STUSB1602_AlertMonitoring_Value.reg_11.b.CC_ATTACHED);

        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum, CAD_HW_Handles[PortNum].cc);

        /* RX mode is enabled */
        HW_IF_RX_Enable(PortNum);
      }
      else  /* CC line is DETACHED */
      {
        /* CAD handle is updated */
        CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
        CAD_HW_Handles[PortNum].cc = CCNONE;

        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum,CAD_HW_Handles[PortNum].cc);

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

      /* If the alert due to a monitoring event is unmasked remove the if statement and put your code */
      if (STUSB1602_Monitoring_Status_Trans_Value.d8 >0)
      {
        __NOP();
      }
    }

    /* Check if a HW FAULT STATUS ALERT detected */
    if ((STUSB1602_AlertMonitoring_Value.reg_0B.b.HW_FAULT_STATUS_AL) && \
      (!STUSB1602_AlertMonitoring_Value.reg_0C.b.HW_FAULT_STATUS_AL_MASK)) 
    {
      /* Check changes occurred in HW FAULT STATUS register and restore the ALERT pin */
      STUSB1602_HW_Fault_Status_Trans_Value = STUSB1602_Hard_Fault_Trans_Status_Get(STUSB1602_I2C_Add(PortNum));

      /* Exit from the alert check procedure */
      AlertAccomplished = 1;

      /* If the alert due to a hardware fault event is unmasked remove then if statement and put your code*/
      if (STUSB1602_HW_Fault_Status_Trans_Value.d8 >0)
      {
        __NOP();
      }
    }

    AlertAttempts--;
    if (AlertAttempts == 0 || AlertAccomplished)
    {
      break;
    }

    /* wait a short time before to try again */
    for(i=0; i<10; i++)
    {
      __NOP();
    }

    return USBPD_OK;
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
      USBPD_BSP_LED_Toggle(PortNum == 0 ? LED_PORT0_CC : LED_PORT1_CC);
      for(i=0; i<400000; i++) __NOP();
    }

  }
  return USBPD_OK;
} 


/**
  * @brief  It enables the data trasmission from microcontroller to STUSB1602 device
  * @param  PortNum The port index
  * @param  pData The pointer to data buffer
  * @param  Size The amount of data to be sent
  * @retval USBPD status
  */
USBPD_StatusTypeDef STUSB16xx_HW_IF_Send_Packet(uint8_t PortNum, uint8_t *pData, uint16_t Size)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* Check if the bus is idle */
  ret = HW_IF_check_bus_idle(PortNum);
  if (ret == USBPD_OK && 1 /* tx ready */)
  {
    /* Set the state to busy*/
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_TX;

    /* Set the SPI in TX mode */
    STUSB16xx_HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);

    HAL_SPI_DMAStop(&Ports[PortNum].hspi);
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
  * @param  PortNum The port index
  * @retval USBPD_StatusTypeDef 
  */
USBPD_StatusTypeDef STUSB16xx_HW_IF_VConnSwap(uint8_t PortNum)
{
  uint32_t STUSB16xx_ACK_timeout = 0xFFFF;
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  if (DPM_Params[PortNum].VconnStatus == USBPD_TRUE)
  {
    Ports[PortNum].VConn = ENABLE;
  }
    else
    {
      Ports[PortNum].VConn = DISABLE;
    }
  if (Ports[PortNum].VConn == DISABLE)
  {
    /* i2c_vconn_swap_turn_on_vconn_req command */
    //ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_ON_VCONN_REQ);    
         ret = (USBPD_StatusTypeDef)STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);    

    /* Check if ACK is got or a timeout is expired */
//    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum))!=PD_VCONN_SWAP_Turn_On_VCONN_Ack) && STUSB16xx_ACK_timeout)
//    {
//      STUSB16xx_ACK_timeout--;
//    }
  }
  else
  {
    /* i2c_vconn_swap_turn_off_vconn_req command */
  //  ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_OFF_VCONN_REQ);    
     ret = (USBPD_StatusTypeDef)STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);    
  
    /* Check if ACK is got or a timeout is expired */
//    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum))!=PD_VCONN_SWAP_Turn_Off_VCONN_Ack) && STUSB16xx_ACK_timeout)
//    {
//      STUSB16xx_ACK_timeout--;
//    } 
  }
  if (STUSB16xx_ACK_timeout == 0)
    ret = USBPD_ERROR;
  
  return ret;
}


/**
  * @brief  Data Role swap management
  * @param  PortNum The port index
  * @retval USBPD_StatusTypeDef 
  */
USBPD_StatusTypeDef STUSB16xx_HW_IF_DataRoleSwap(uint8_t PortNum)
{
  uint32_t STUSB16xx_ACK_timeout = 0xFFFF;
  USBPD_StatusTypeDef ret = USBPD_ERROR;

  if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_DFP)
  {
    /* i2c_dr_swap_port_change_2_ufp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ);    
      
//    /* Check if ACK is got or a timeout is happen */
//    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum)) != PD_DR_SWAP_Port_Change_2_UFP_Ack) && STUSB16xx_ACK_timeout)
//    {
//      STUSB16xx_ACK_timeout--;
//    }
  }
  else if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_UFP)
  {
    /* i2c_dr_swap_port_change_2_dfp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ);    
      
//    /* Check if ACK is got or a timeout is happen */
//    while ((STUSB1602_PD_TypeC_HandShake_Get(STUSB1602_I2C_Add(PortNum)) != PD_DR_SWAP_Port_Change_2_DFP_Ack) && STUSB16xx_ACK_timeout)
//    {
//      STUSB16xx_ACK_timeout--;
//    }
  }
  else
    ret = USBPD_ERROR;

  if (STUSB16xx_ACK_timeout == 0)
    ret = USBPD_ERROR;

  return ret;
}


/**
  * @brief  It sets the VBus monitoring reference voltage as well as the upper and lower tolerance threshold
  * @param  PortNum The port index
  * @param  VBus It is the VBus monitoring reference voltage (expressed in mV)
  * @param  Hset It is the upper tolerance (expressed in % and >= 5%)
  * @param  Lset It is the lower tolerance (expressed in % and >= 5%)
  * @retval USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef STUSB16xx_HW_IF_Set_VBus_Monitoring(uint8_t PortNum, uint16_t VBus, uint8_t Hset, uint8_t Lset)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;

  /* check for parameters*/
  if (Hset > 15) Hset = 15;
  if (Lset > 15) Lset = 15;

  /* Sets the VBUS_SELECT DAC reference for VBUS sensing (bit7:0 0x21) */
  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), VBus);

  /* Sets the VBUS_VShift_High (bit7:4 0x22) */
  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), Hset);

  /* Sets the VBUS_VShift_Low (bit3:0 0x22) */
  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), Lset);

  return ret;
}

/**
  * @brief  Get Sink Resistors State
  * @param  PortNum The port index
  * @retval USBPD_FALSE of USBPD_TRUE
  */
uint8_t USBPD_16xx_IsResistor_SinkTxOk(uint8_t PortNum)
{
  uint8_t TX_OK;

  USBPD_StatusTypeDef ret = USBPD_ERROR;
  /* try to acquire the communication resource to avoid the conflict */
  ret = HW_IF_COMM_WAIT(PortNum, COMM_TO_DEFAULT);
  if (ret != USBPD_OK)
  {
    return USBPD_FALSE;
  }
  TX_OK = STUSB1602_Sink_Power_State_Get(STUSB1602_I2C_Add(PortNum));
  HW_IF_COMM_RELEASE(PortNum);

  if ( TX_OK == Pwr_3_0_SNK)
  {
    return USBPD_TRUE;
  }
  else
  {
    return USBPD_FALSE;
  }
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
