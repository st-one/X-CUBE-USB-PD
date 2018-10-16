/**
  ******************************************************************************
  * @file    fusb305.c
  * @author  MCD Application Team
  * @version $VERSION$
  * @date    $DATE$
  * @brief   This file includes the driver for FUSB305.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "fusb305.h"
#include "usbpd_trace.h"
#include "usbpd_core.h"
#include "usbpd_tcpci.h"

/** @addtogroup TCPC_Driver
  * @{
  */

/** @addtogroup FUSB305_TCPC
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/** @defgroup FUSB305_TCPC_Private_Typedef FUSB305 Private Typedef
 * @{
 */
TCPC_DrvTypeDef  fusb305_tcpc_drv = 
{
  fusb305_tcpc_init,
  fusb305_tcpc_get_cc,
  fusb305_tcpc_get_power_status,
  fusb305_tcpc_get_fault_status,
  fusb305_tcpc_set_fault_status,
  fusb305_tcpc_get_vbus_level,
  fusb305_tcpc_set_vbus_level,
  fusb305_tcpc_set_cc,
  fusb305_tcpc_set_polarity,
  fusb305_tcpc_set_vconn,
  fusb305_tcpc_set_msg_header,
  fusb305_tcpc_set_rx_state,
  fusb305_tcpc_set_sop_supported,
  fusb305_tcpc_get_message,
  fusb305_tcpc_alert,
  fusb305_tcpc_clear_alert,
  fusb305_tcpc_transmit,
  fusb305_tcpc_set_bist_test_data,
  fusb305_tcpc_SinkTxNG,
  fusb305_tcpc_SinkTxOK,
  fusb305_tcpc_IfSinkTxOk,
};

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/** @defgroup FUSB305_TCPC_Private_Defines FUSB305 Private Defines
 * @{
 */
#if defined(__GNUC__)
/* GNU Compiler */
#elif defined(__CC_ARM)
/* ARM Compiler */
#pragma anon_unions
#elif defined(__ICCARM__)
/* IAR Compiler */
#endif


#if !defined(USBPD_PORT_COUNT)
#define USBPD_PORT_COUNT 1
#endif

#if defined(_TRACE)
#define FUSB307_DEBUG_NONE     (0)
#define FUSB307_DEBUG_LEVEL_0  (1 << 0)
#define FUSB307_DEBUG_LEVEL_1  (1 << 1)
#define FUSB307_DEBUG_LEVEL_2  (1 << 2)
#endif /* _TRACE */

#define CC_DEBOUNCE_TIMER               4U      /**< tCCDebounce threshold = 4 ms  */

#define USBPD_TCPM_MAX_RX_BUFFER_SIZE         (30U)    /*!< Maximum size of Rx buffer */

/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/** @defgroup FUSB305_TCPC_Private_Macros FUSB305 Private Macros
 * @{
 */
#define PD_HEADER_COUNT(__HEADER__)   (((__HEADER__) >> 12) & 7)
#define PD_HEADER_TYPE(__HEADER__)    ((__HEADER__) & 0xF)
#define PD_HEADER_ID(__HEADER__)      (((__HEADER__) >> 9) & 7)

#if defined(_TRACE)
#define FUSB307_DEBUG_TRACE(__PORT__, _LEVEL_, __STRING__) \
  if(FUSB307_DebugLevel & (_LEVEL_)) { USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (__PORT__), 0, (__STRING__), sizeof(__STRING__)-1); }
#else
#define FUSB307_DEBUG_TRACE(__PORT__, _LEVEL_, __STRING__)
#endif /* _TRACE */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup FUSB305_TCPC_Private_Variables FUSB305 Private Variables
 * @{
 */

#if defined(_TRACE)
uint8_t FUSB307_DebugLevel = FUSB307_DEBUG_LEVEL_0;
#endif /* _TRACE */

typedef union {
  uint8_t byte[21];
  struct {
    union {
      uint8_t VCONN_OCP; /* 0xA0 */
      struct {
        uint8_t OCP_Current:3;
        uint8_t OCP_Range:1;
        uint8_t Reserved:4;
      };
    };
    uint8_t Vendor_Reserved1;
    union {
      uint8_t RESET; /* 0xA2 */
      struct {
        uint8_t SW_RST:1;
        uint8_t PD_RST:1;
        uint8_t R_Reserved:6;
      };
    };
    uint8_t Vendor_Reserved2;
    union {
      uint8_t GPIO1_CFG; /* 0xA4 */
      struct {
        uint8_t GPO1_EN:1;
        uint8_t GPI1_EN:1;
        uint8_t GPO1_VAL:1;
        uint8_t GPIO1_Reserved:5;
      };
    };
    union {
      uint8_t GPIO2_CFG; /* 0xA5 */
      struct {
        uint8_t GPO2_EN:1;
        uint8_t GPI2_EN:1;
        uint8_t GPO2_VAL:1;
        uint8_t FR_SWAP_EN:1;
        uint8_t GPIO2_Reserved:4;
      };
    };
    union {
      uint8_t GPIO_STAT; /* 0xA6 */
      struct {
        uint8_t GPI1_VAL:1;
        uint8_t GPI2_VAL:1;
        uint8_t GPIO_Reserved:6;
      };
    };
    union {
      uint8_t DRPTOGGLE; /* 0xA7 */
      struct {
        uint8_t DRPTOGGLE_VAL:2;
        uint8_t DRP_Reserved:6;
      };
    };
    union {
      uint8_t TOGGLE_SM; /* 0xA8 */
      struct {
        uint8_t TOGGLE_SM_VAL:5;
        uint8_t TOGGLE_SM_Reserved:3;
      };
    };
    uint8_t Vendor_Reserved3[7];/* 0xA9 -> 0xAF */
    union {
      uint8_t SINK_TRANSMIT; /* 0xB0 */
      struct {
        uint8_t TXSOP:3;
        uint8_t SNK_TX_Reserved1:1;
        uint8_t RETRY_CNT:2;
        uint8_t DIS_SNK_TX:1;
        uint8_t SNK_TX_Reserved2:1;
      };
    };
    union {
      uint8_t SRC_FRSWAP; /* 0xB1 */
      struct {
        uint8_t FR_SWAP:1;
        uint8_t ManualSnkEn:1;
        uint8_t FrSwapSnkDelay:2;
        uint8_t SRC_Reserved:4;
      };
    };
    union {
      uint8_t SNK_FRSWAP; /* 0xB2 */
      struct {
        uint8_t EnFrSwapDtct:1;
        uint8_t SNK_Reserved:7;
      };
    };
    union {
      uint8_t ALERT_VD; /* 0xB3 */
      struct {
        uint8_t I_SWAP_RX:1;
        uint8_t I_SWAP_TX:1;
        uint8_t I_OTP:1;
        uint8_t I_VDD_DTCT:1;
        uint8_t I_GPI1:1;
        uint8_t I_GPI2:1;
        uint8_t I_DISCH_SUCC:1;
        uint8_t I_ALERT_Reserved:1;
      };
    };
    union {
      uint8_t ALERT_VD_MASK; /* 0xB4 */
      struct {
        uint8_t M_SWAP_RX:1;
        uint8_t M_SWAP_TX:1;
        uint8_t M_OTP:1;
        uint8_t M_VDD_DTCT:1;
        uint8_t M_GPI1:1;
        uint8_t M_GPI2:1;
        uint8_t M_DISCH_SUCC:1;
        uint8_t M_ALERT_Reserved:1;
      };
    };
  };
} regVendorInfo_t;

static struct fusb305_chip_state {
  DeviceReg_t     Registers;                /*!< Variable holding the current status of the device registers */
  regVendorInfo_t Vendor;
  USBPD_PortPowerRole_TypeDef PowerRole;    /*!< Keep the current power role */
  USBPD_PortDataRole_TypeDef DataRole ;     /*!< Keep the current data role */
  TCPC_CC_Pull_TypeDef  CC_Pull;            /*!< Keep the current CC pull */
  TCPC_RP_Value_TypeDef RP_Value;           /*!< Keep the current RP Value */
  uint8_t       VConnPresence;              /*!< Presence of VCONN */
  uint8_t       TogglingEnable;             /*!< Toggling activation */
  uint8_t       (*IsSwapOngoing)(uint8_t);  /*!< function used by PHY to avoid Idle BUS check when a swap is ongoing */
} state[USBPD_PORT_COUNT];

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup FUSB305_TCPC_Private_Functions FUSB305 Private Functions
 * @{
 */
static void                 InitializeRegisters(uint32_t Port);
static USBPD_StatusTypeDef  tcpc_set_power(uint32_t Port, USBPD_FunctionalState State, TCPC_hard_reset HardReset);
static USBPD_StatusTypeDef  tcpc_set_alert_mask(uint32_t Port, uint8_t Pull, USBPD_FunctionalState State);
static USBPD_StatusTypeDef  tcpc_init_power_status_mask(uint32_t Port);
static void                 tcpc_set_pin_role(uint32_t Port, uint8_t Pull, USBPD_FunctionalState Connection);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup FUSB305_TCPC_Exported_Functions FUSB305 Exported Functions
 * @{
 */
/**
  * @brief  Initialize FUSB305 device
  * @param  Port port number value
  * @param  Role Power role can be one of the following values:
  *         @arg @ref USBPD_PORTPOWERROLE_SNK
  *         @arg @ref USBPD_PORTPOWERROLE_SRC
  *         @arg @ref USBPD_PORTPOWERROLE_DRP_SRC
  *         @arg @ref USBPD_PORTPOWERROLE_DRP_SNK
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_init(uint32_t Port, USBPD_PortPowerRole_TypeDef Role, uint8_t ToggleRole, uint8_t (*IsSwapOngoing)(uint8_t))
{
  USBPD_StatusTypeDef usbpd_status = USBPD_FAIL;
  uint8_t power_status = 1;

  /* Reset FUSB305 */
  usbpd_status = USBPD_TCPCI_WriteRegister(Port, 0xA2, (uint8_t*)&power_status, 2);

  /* all other variables assumed to default to 0 */
  InitializeRegisters(Port);

  /* Save the CAD callback to check if PR swap is ongoing */
  state[Port].IsSwapOngoing        = IsSwapOngoing;

  /* 
   * TCPM shall check the state of the TCPC Initialization Status bit when it starts or resets. 
   * TCPM shall not start normal operation until the TCPC Initialization Status bit is cleared.
   */
  while (1) 
  {
    usbpd_status = fusb305_tcpc_get_power_status(Port, &power_status);
    /*
     * If read succeeds and the uninitialized bit is clear, then
     * initalization is complete, clear all alert bits and write
     * the initial alert mask.
     */
    if ((usbpd_status == USBPD_OK) && !(power_status & TCPC_REG_POWER_STATUS_MASK_TCPC_INIT)) 
    {
      /* Initialize power role */
      state[Port].PowerRole     = USBPD_PORTPOWERROLE_SNK;
      state[Port].DataRole      = USBPD_PORTDATAROLE_UFP;
      state[Port].CC_Pull       = TYPEC_CC_OPEN;
      state[Port].VConnPresence = TYPEC_CC_OPEN;
      
      if (usbpd_status == USBPD_OK)
      {
        TCPC_CC_Pull_TypeDef cc_value  = TYPEC_CC_OPEN;
        /* Initialize power_status_mask */
        if (tcpc_init_power_status_mask(Port) != USBPD_OK)
        {
          goto exit;
        }
        
        /* Initialize alert mask*/
        if (tcpc_set_alert_mask(Port, TYPEC_CC_OPEN, USBPD_DISABLE) != USBPD_OK)
        {
          goto exit;
        }

        /* Disable all the VD interrupt */
        state[Port].Vendor.ALERT_VD_MASK = 0;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT_VD_MASK, (uint8_t*)&state[Port].Vendor.ALERT_VD_MASK, 1);

        /*  
            GPIO1 can be used to prevent the FPF3695 from closing the sink path automatically when in dead battery condition.
            It allows you to disable the entire FPF3695, regardless of attach state.
            Most systems are ok with closing the path on attach, so GPIO1 should be driven low (enabled).
        */
        state[Port].Vendor.GPO1_EN = 1;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_GPIO1_CFG, (uint8_t*)&state[Port].Vendor.GPIO1_CFG, 1);

        /* Disable RX_DETECT in waiting for connection */
        state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT = 0;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);

        /* Disable SOURCE VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1) != USBPD_OK)
        {
          goto exit;
        }

        /* Disable SOURCE VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_VBUS_DETECT;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1) != USBPD_OK)
        {
          goto exit;
        }

        /* Reset Power control register */
        state[Port].Registers.Control.s.u5.POWER_CONTROL = 0;
        state[Port].Registers.Control.s.u5.b5.DIS_VALRM   = 1;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1) != USBPD_OK)
        {
          goto exit;
        }
          /* Initialize VBUS Threshold */
        state[Port].Registers.VBUS.s.VBUS_VOLTAGE_ALARM_HI_CFG   = 840;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG, (uint8_t*)&state[Port].Registers.VBUS.s.VBUS_VOLTAGE_ALARM_HI_CFG, 2) != USBPD_OK)
        {
          goto exit;
        }
        
        state[Port].Registers.VBUS.s.VBUS_VOLTAGE_ALARM_LO_CFG   = 32;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG, (uint8_t*)&state[Port].Registers.VBUS.s.VBUS_VOLTAGE_ALARM_LO_CFG, 2) != USBPD_OK)
        {
          goto exit;
        }

        /* Set ROLE_CONTROL for TCPC */
        state[Port].TogglingEnable       = ToggleRole;
        switch(Role)
        {
          case USBPD_PORTPOWERROLE_SNK:         /*!< Sink                         */
            cc_value = TYPEC_CC_RD;
            break;
          case USBPD_PORTPOWERROLE_SRC:         /*!< Source                       */
            cc_value = TYPEC_CC_RP;
            break;
        default:
            usbpd_status = USBPD_ERROR;
            goto exit;
        }

        if (fusb305_tcpc_set_cc(Port, cc_value, USBPD_DISABLE) != USBPD_OK)
        {
          goto exit;
        }
        
        /* Start DRP toggling */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_LOOK4CONNECTION;
        if (USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1) != USBPD_OK)
        {
          goto exit;
        }
        state[Port].Registers.Alerts.word[0] = TCPC_REG_ALERT_CLEAR_ALL;
        usbpd_status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT, (uint8_t*)&state[Port].Registers.Alerts.word[0], 2);
        
        break;
      }
      else
      {
        return usbpd_status;
      }
    }
  }

exit:

  return usbpd_status;
}

/**
  * @brief  Get CC line for PD connection
  * @param  Port port number value
  * @param  CC1_Level Pointer of status of the CC1 line
  * @param  CC2_Level Pointer of status of the CC2 line
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_get_cc(uint32_t Port, uint32_t *CC1_Level, uint32_t *CC2_Level)
{
  USBPD_StatusTypeDef status = USBPD_BUSY;

  /* Start a delay for CC debounce */
  USBPD_TCPCI_Delay(CC_DEBOUNCE_TIMER);
  
  /* If TCPC read fails, return error */
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_CC_STATUS, (uint8_t*)&state[Port].Registers.Status.u1.CC_STATUS, 1);
  
#if defined(_TRACE)
  if (*CC1_Level != 0xFF)
  {
    uint8_t tab[10] = {0};
    uint8_t size = sprintf((char*)tab,"CC_S2=%2X", *&state[Port].Registers.Status.u1.CC_STATUS);
    FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
  }
#endif /* _TRACE */
  
  /* Check for CC connection only if TCPC found a connection */
  if (state[Port].Registers.Status.u1.b1.LOOK4CON != 1)
  {
    *CC1_Level = state[Port].Registers.Status.u1.b1.CC1_STAT;
    *CC2_Level = state[Port].Registers.Status.u1.b1.CC2_STAT;

    /* Check if need to keep value of RA */
    if ((TYPEC_CC_RA != state[Port].Registers.Control.s.u3.b3.CC1_TERM)
      && (TYPEC_CC_RA != state[Port].Registers.Control.s.u3.b3.CC2_TERM))
    { 
      /* Reset VCONN Presence */
      state[Port].VConnPresence = TYPEC_CC_OPEN;
    }
    
    /*
     * If status is not open, then OR in termination to convert to
     * enum @ref tcpc_cc_voltage_status.
     */
    status = USBPD_OK;
    if (state[Port].IsSwapOngoing(Port) == 0)
    {
      /* Check CC1 pin */
      if (*CC1_Level != TYPEC_CC_VOLT_OPEN)
      {
        if (((state[Port].TogglingEnable == 0) &&
                ((TYPEC_CC_RD == state[Port].Registers.Control.s.u3.b3.CC1_TERM)||(state[Port].Registers.Status.u1.b1.CON_RES == 1)))
            ||((state[Port].TogglingEnable == 1) &&
                (state[Port].Registers.Status.u1.b1.CON_RES == 1)))
        {
          *CC1_Level |= (1 << 2);
          /* Save current power role */
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SNK;
          state[Port].DataRole  = USBPD_PORTDATAROLE_UFP;
          state[Port].CC_Pull   = TYPEC_CC_RD;
        }
        else
        {
          /* Check that on another pin than a RA is present */
          if (TYPEC_CC_VOLT_RA == *CC2_Level)
          {
            /* RA detected on pin CC2 */
            state[Port].VConnPresence = TYPEC_CC_RA;
          }
          /* Save current power role */
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SRC;
          state[Port].DataRole  = USBPD_PORTDATAROLE_DFP;
          state[Port].CC_Pull   = TYPEC_CC_RP;
        }
      }
      
      /* Check CC2 pin */
      if (*CC2_Level != TYPEC_CC_VOLT_OPEN)
      {
        if (((state[Port].TogglingEnable == 0) &&
                ((TYPEC_CC_RD == state[Port].Registers.Control.s.u3.b3.CC2_TERM)||(state[Port].Registers.Status.u1.b1.CON_RES == 1)))
            ||((state[Port].TogglingEnable == 1) &&
                (state[Port].Registers.Status.u1.b1.CON_RES == 1)))
        {
          *CC2_Level |= (1 << 2);
          /* Save current power role */
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SNK;
          state[Port].DataRole  = USBPD_PORTDATAROLE_UFP;
          state[Port].CC_Pull   = TYPEC_CC_RD;
        }
        else
        {
          /* Check that on another pin than a RA is present */
          if (TYPEC_CC_VOLT_RA == *CC1_Level)
          {
            /* RA detected on pin CC1 */
            state[Port].VConnPresence = TYPEC_CC_RA;
          }
          /* Save current power role */
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SRC;
          state[Port].DataRole  = USBPD_PORTDATAROLE_DFP;
          state[Port].CC_Pull   = TYPEC_CC_RP;
        }
      }
    }
  }

  return status;
}

/**
  * @brief  Set CC line for PD connection
  * @param  Port port number value
  * @param  Pull Power role can be one of the following values:
  *         @arg @ref TYPEC_CC_RA
  *         @arg @ref TYPEC_CC_RP
  *         @arg @ref TYPEC_CC_RD
  *         @arg @ref TYPEC_CC_OPEN
  * @param  State  State of the connection
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_cc(uint32_t Port, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State)
{
  /* Save current power role */
  if (TYPEC_CC_RP == Pull)
  {
    state[Port].PowerRole = USBPD_PORTPOWERROLE_SRC;
    state[Port].DataRole  = USBPD_PORTDATAROLE_DFP;
  }
  else
  {
    state[Port].PowerRole = USBPD_PORTPOWERROLE_SNK;
    state[Port].DataRole  = USBPD_PORTDATAROLE_UFP;
  }
  state[Port].CC_Pull = Pull;

  /* Set the role of each CC pin */
  tcpc_set_pin_role(Port, Pull, State);

  /* Check if PD communication is enabled */
  if (state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT != 0)
  {
    /* Udpate ALERT mask */
    tcpc_set_alert_mask(Port, state[Port].PowerRole, USBPD_ENABLE);
  }

  /* Update role in TCPC PD header */
  fusb305_tcpc_set_msg_header(Port, state[Port].PowerRole, state[Port].DataRole);
  
  return USBPD_OK;
}

/**
  * @brief  Set the polarity of the CC lines
  * @param  Port port number value
  * @param  Polarity Polarity
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_polarity(uint32_t Port, uint8_t Polarity)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT = Polarity;

  status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);

#if 0
  /* Check if TCPC role is set to source */
  if (state[Port].PowerRole == USBPD_PORTPOWERROLE_SRC)
  {
    state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK = 0xFF;
    status |= USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_STATUS_MASK , &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  }
  else
  {
    state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK = TCPC_REG_POWER_STATUS_MASK_VBUS_PRES;
    status |= USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_STATUS_MASK , &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  }
#endif

  return status;
}

/**
  * @brief  Enable or disable VCONN
  * @param  Port port number value
  * @param  State Activation or deactivation of VCONN
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_vconn(uint32_t Port, USBPD_FunctionalState State)
{
  state[Port].Registers.Control.s.u5.b5.EN_VCONN = 0;
  if (State)
  {
    state[Port].Registers.Control.s.u5.b5.EN_VCONN = 1;
  }
  USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);

  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  
  if (State)
  {
    while (state[Port].Registers.Status.u2.b2.VCONN_VAL == 0)
      USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  }
  else
  {
    while (state[Port].Registers.Status.u2.b2.VCONN_VAL == 1)
      USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  }
 
  return USBPD_OK;
}

/**
  * @brief  Set power and data role et PD message header
  * @param  PortNum   port number value
  * @param  PowerRole Power role
  * @param  DataRole  Data role
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_msg_header(uint32_t PortNum, USBPD_PortPowerRole_TypeDef PowerRole, USBPD_PortDataRole_TypeDef DataRole)
{
  state[PortNum].Registers.FrameInfo.u1.MESSAGE_HEADER_INFO = TCPC_REG_MSG_HEADER_INFO_SET(DataRole, PowerRole);

  return USBPD_TCPCI_WriteRegister(PortNum, TCPC_REG_MSG_HEADER_INFO, &state[PortNum].Registers.FrameInfo.u1.MESSAGE_HEADER_INFO, 1);
}

/**
  * @brief  Enable or disable PD reception
  * @param  Port          Port number value
  * @param  Enable        Activation or deactivation of RX
  * @param  SupportedSOP  Supported SOP by PRL
  * @param  HardReset     Hard reset status based on @ref TCPC_hard_reset
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_rx_state(uint32_t Port, TCPC_CC_Pull_TypeDef Pull, USBPD_FunctionalState State, uint32_t SupportedSOP, TCPC_hard_reset HardReset)
{
  USBPD_StatusTypeDef status = USBPD_OK;
#if defined(_TRACE)
  {
    uint8_t tracedata[18] = {0};
    sprintf((char*)tracedata,"state=%d", State);
    FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tracedata);
  }
#endif

  /* Just enable RX after receiving a hard reset */
  if ((TCPC_HARD_RESET_RECEIVED == HardReset) || (TCPC_HARD_RESET_SENT == HardReset))
  {
    /* Enable RX */
    state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT = SupportedSOP;
    state[Port].Registers.FrameInfo.u2.b2.EN_HRD_RST  = 1;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);

    /* Set role of CC pins */
    //fusb305_tcpc_set_cc(Port, state[Port].CC_Pull, State);

    return status;
  }

  /* Check if TCPC role is set to source */
  if (TYPEC_CC_RP == Pull)
  {
    state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK = 0xFF;
    status |= USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_STATUS_MASK , &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  }
  else
  {
    state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK = TCPC_REG_POWER_STATUS_MASK_VBUS_PRES;
    status |= USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_STATUS_MASK , &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  }

  /* If enable, then set RX detect for SOP */
  if (State)
  {
    /* Set role of CC pins */
    fusb305_tcpc_set_cc(Port, Pull, State);
    
    if (USBPD_OK == tcpc_set_power(Port, State, HardReset))
    {
      /* Update ALERT mask */
      tcpc_set_alert_mask(Port, state[Port].PowerRole, USBPD_ENABLE);

      /* Enable RX */
      state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT = SupportedSOP;
      state[Port].Registers.FrameInfo.u2.b2.EN_HRD_RST  = 1;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);
    }
    else
    {
#if 1
      if (state[Port].TogglingEnable == 1)
      {
        /* Change power role to restart DRP toggling */
        if (TYPEC_CC_RP == Pull)
        {
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SNK;
          state[Port].DataRole  = USBPD_PORTDATAROLE_UFP;
          state[Port].CC_Pull   = TYPEC_CC_RD;
        }
        else
        {
          state[Port].PowerRole = USBPD_PORTPOWERROLE_SRC;
          state[Port].DataRole  = USBPD_PORTDATAROLE_DFP;
          state[Port].CC_Pull   = TYPEC_CC_RP;
        }
      }
#endif
      /* Set role of CC pins */
      fusb305_tcpc_set_cc(Port, state[Port].CC_Pull, USBPD_DISABLE);

      /* Reset ALERT MASK */
      tcpc_set_alert_mask(Port,  TYPEC_CC_OPEN, USBPD_DISABLE);
      
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_LOOK4CONNECTION;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

      status =  USBPD_FAIL;
    }
  }
  else
  {
    /* Detach has been detected, reset BIST data is case of activation */
    state[Port].Registers.Control.s.u2.b2.BIST_TMODE = 0;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);

    /* Disable VBUS */
    status = tcpc_set_power(Port, USBPD_DISABLE, TCPC_HARD_RESET_NONE);

    /* Reset ALERT MASK */
    tcpc_set_alert_mask(Port,  TYPEC_CC_OPEN, USBPD_DISABLE);

    /* Disable RX */
    state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT = 0;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);

    /* Disable VBUS detect */
    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_VBUS_DETECT;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

#if 1
#if 0
    if ((state[Port].TogglingEnable == 1) && (status == USBPD_OK))
#else
    if (state[Port].TogglingEnable == 1)
#endif
    {
      /* Change power role to restart DRP toggling */
      if (TYPEC_CC_RP == state[Port].CC_Pull)
      {
        state[Port].PowerRole = USBPD_PORTPOWERROLE_SNK;
        state[Port].DataRole  = USBPD_PORTDATAROLE_UFP;
        state[Port].CC_Pull   = TYPEC_CC_RD;
      }
      else
      {
        state[Port].PowerRole = USBPD_PORTPOWERROLE_SRC;
        state[Port].DataRole  = USBPD_PORTDATAROLE_DFP;
        state[Port].CC_Pull   = TYPEC_CC_RP;
      }
    }
#endif

    /* Set CC functionality */
    fusb305_tcpc_set_cc(Port, state[Port].CC_Pull, USBPD_DISABLE);

    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_LOOK4CONNECTION;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
  }

  return status;
}

/**
  * @brief  Set SOP supported
  * @param  PortNum       PortNum number value
  * @param  SupportedSOP  Supported SOP by PRL
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_sop_supported(uint32_t Port, uint32_t SupportedSOP)
{
  USBPD_StatusTypeDef status = USBPD_OK;

  /* Enable RX */
  state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT = SupportedSOP;
  state[Port].Registers.FrameInfo.u2.b2.EN_HRD_RST  = 1;
  USBPD_TCPCI_WriteRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);
  return status;
}

/**
  * @brief  Retrieve the PD message 
  * @param  Port port number value
  * @param  Payload Pointer on the payload
  * @param  Type Pointer on the message type
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_get_message(uint32_t Port, uint8_t *Payload, uint8_t *Type)
{
  return USBPD_TCPCI_ReceiveBuffer(Port, TCPC_REG_RX_BYTE_COUNT, (uint8_t *)Payload, Type);
}

/**
  * @brief  Transmit the PD message 
  * @param  Port port number value
  * @param  Type Message type
  * @param  Header Header of the PD message
  * @param  pData Pointer on the data message
  * @param  RetryNumber Number of retry
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_transmit(uint32_t Port, USBPD_SOPType_TypeDef Type, uint16_t Header, const uint8_t *pData, uint32_t RetryNumber)
{
  USBPD_StatusTypeDef status = USBPD_ERROR;
  if ((USBPD_SOPTYPE_HARD_RESET != Type) && (USBPD_SOPTYPE_BIST_MODE_2 != Type) && (USBPD_SOPTYPE_CABLE_RESET != Type))
  {
    uint32_t count = 4 * PD_HEADER_COUNT(Header);

    if (count > USBPD_TCPM_MAX_RX_BUFFER_SIZE)
    {
      return status;
    }

    /* 
    * Writing the TRANSMIT_BUFFER:
    * - TCPC_REG_TX_BYTE_COUNT register address
    * - Transmit Byte Count
    * - Transmit Header
    * - Data
    */
    state[Port].Registers.TXFrame.u.TRANSMIT_BYTE_COUNT = (count + 2);
    status = USBPD_TCPCI_SendTransmitBuffer(Port, TCPC_REG_TX_BYTE_COUNT, (count + 2), Header, (uint8_t *)pData);
    
    if (status == USBPD_OK)
    {
      state[Port].Registers.TXFrame.u.TRANSMIT = TCPC_REG_TRANSMIT_SET(Type, RetryNumber);
      status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TRANSMIT, &state[Port].Registers.TXFrame.u.TRANSMIT, 1);
    }
  }
  else
  {
    /* Send Hard Reset, Cable Reset, or BIST Carrier Mode 2 signaling */
    state[Port].Registers.TXFrame.u.TRANSMIT_BYTE_COUNT = 0;
    status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TX_BYTE_COUNT, &state[Port].Registers.TXFrame.u.TRANSMIT_BYTE_COUNT, 1);
    state[Port].Registers.TXFrame.u.TRANSMIT = TCPC_REG_TRANSMIT_SET(Type, 0);
    status |= USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TRANSMIT, &state[Port].Registers.TXFrame.u.TRANSMIT, 1);
  }

  return status;
}

/**
  * @brief  Get Power status level
  * @param  Port port number value
  * @param  PowerStatus Pointer on Power status
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_get_power_status(uint32_t Port, uint8_t *PowerStatus)
{
  USBPD_StatusTypeDef usbpd_status = USBPD_OK;

  usbpd_status = USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  *PowerStatus = state[Port].Registers.Status.u2.POWER_STATUS;

#if defined(_TRACE)
  uint8_t tab[10] = {0};
  uint8_t size = sprintf((char*)tab,"power=%2x", *PowerStatus);
  FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
#endif /* _TRACE */

  return usbpd_status;
}

/**
  * @brief  Get fault status register
  * @param  Port port number value
  * @param  FaultStatus Pointer on Fault status
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_get_fault_status(uint32_t Port, uint8_t *FaultStatus)
{
  USBPD_StatusTypeDef usbpd_status = USBPD_OK;

  usbpd_status = USBPD_TCPCI_ReadRegister(Port, TCPC_REG_FAULT_STATUS, &state[Port].Registers.Status.u3.FAULT_STATUS, 1);
  *FaultStatus = state[Port].Registers.Status.u3.FAULT_STATUS;

  return usbpd_status;
}

/**
  * @brief  Set fault status register
  * @param  Port        port number value
  * @param  FaultStatus Fault status
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_fault_status(uint32_t Port, uint8_t FaultStatus)
{
  USBPD_StatusTypeDef usbpd_status = USBPD_OK;

  state[Port].Registers.Status.u3.FAULT_STATUS = FaultStatus;
  usbpd_status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_FAULT_STATUS, &FaultStatus, 1);

  return usbpd_status;
}

/**
  * @brief  Get VBUS level
  * @param  Port port number value
  * @param  VBUSLevel   Pointer on VBUS level
  * @param  VBUSVoltage Pointer on VBUS Voltage (in mV)
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_get_vbus_level(uint32_t Port, uint8_t *VBUSLevel, uint16_t *VBUSVoltage)
{
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  *VBUSLevel = state[Port].Registers.Status.u2.b2.VBUS_VAL;

  /* Get VBUS measurement */
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_VBUS_VOLTAGE, (uint8_t*)&state[Port].Registers.VBUS.s.u.VBUS_VOLTAGE, 2);
  *VBUSVoltage = state[Port].Registers.VBUS.s.u.VBUS_VOLTAGE * 25;

#if defined(_TRACE)
  static volatile uint8_t previous_vbus = 0;
  static volatile uint16_t previous_vbus_voltage = 0;
  uint8_t tab[15] = {0};

  if (previous_vbus != *VBUSLevel)
  {
    previous_vbus = *VBUSLevel;
    sprintf((char*)tab,"Vbus=%2x", previous_vbus);
    FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
  }

  if (previous_vbus_voltage != *VBUSVoltage)
  {
    previous_vbus_voltage = *VBUSVoltage;
    sprintf((char*)tab,"VbusVolt=%d", previous_vbus_voltage);
    FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
  }
#endif /* _TRACE */

  return USBPD_OK;
}

/**
  * @brief  Set VBUS level
  * @param  Port port number value
  * @param  Enable Enable or disable VBUS level
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_vbus_level(uint32_t Port, USBPD_FunctionalState State)
{
  uint8_t vbus_level = 0;
  state[Port].Registers.Control.s.u5.POWER_CONTROL = 0;
  if (USBPD_ENABLE == State)
  {
    uint8_t TimeoutSink = 0;
    /* Enable SINK or SOURCE VBUS*/
    if (state[Port].PowerRole == USBPD_PORTPOWERROLE_SRC)
    {
      /* Enable VCONN */
      state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 1;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);

      /* Disable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

      /* Add delay between disable SINK VBUS and enable SRC VBUS */
      USBPD_TCPCI_Delay(2);

      /* Enable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_SRC_VBUS_DEFAULT;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    }
    else
    {
      /* Disable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      /* Enable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_ENABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    }

    /* Wait for VBUS ready */
    fusb305_tcpc_get_vbus_level(Port, &vbus_level, NULL);
    while (vbus_level != 1)
    {
      /* Add a delay for CC debounce */
      USBPD_TCPCI_Delay(4);

      USBPD_TCPCI_ReadRegister(Port, TCPC_REG_CC_STATUS, (uint8_t*)&state[Port].Registers.Status.u1.CC_STATUS, 1);
      /* Check if CC line has been disconnected */
      /* or if no more VBUS after CC_DEBOUNCE_TIMER x 3 (around 12ms) */
      if (((TYPEC_CC_VOLT_OPEN == state[Port].Registers.Status.u1.b1.CC1_STAT) 
           && (state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT == 0))
          || ((TYPEC_CC_VOLT_OPEN == state[Port].Registers.Status.u1.b1.CC2_STAT) 
              && (state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT == 1))
        || ((state[Port].PowerRole == USBPD_PORTPOWERROLE_SNK) && (TimeoutSink > 250)))
      {
        /* Disable SOURCE VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
        /* Disable SINK VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
        
        /* Disable VCONN */
        state[Port].Registers.Control.s.u5.b5.EN_VCONN    = 0;
        state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 0;
        state[Port].Registers.Control.s.u5.b5.DIS_VALRM   = 1;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);
        /* Disable BIST mode */
        state[Port].Registers.Control.s.u2.b2.BIST_TMODE = 0;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);
        return USBPD_FAIL;
      }
      TimeoutSink++;
      fusb305_tcpc_get_vbus_level(Port, &vbus_level, NULL);
    }
  }
  else
  {
    uint16_t vbus_voltage = 0;
    /* Enable Force discharge */
    state[Port].Registers.Control.s.u5.b5.FORCE_DISCH  = 1;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);

    /* Disable SOURCE VBUS */
    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

    /* Disable SINK VBUS */
    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

    /* Wait for VBUS ready */
    fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
    while ((vbus_level != 0) || (vbus_voltage > 800)) {
      /* Disable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      /* Disable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
    };

    state[Port].Registers.Control.s.u5.b5.FORCE_DISCH    = 0;
    state[Port].Registers.Control.s.u5.b5.DIS_VALRM      = 1;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);
  }

  return USBPD_OK;
}

/**
  * @brief  Set VBUS level
  * @param  Port        Port number value
  * @param  State       Enable or disable VBUS level
  * @param  HardReset   Hard reset status based on @ref TCPC_hard_reset
  * @retval USBPD status
  */
static USBPD_StatusTypeDef tcpc_set_power(uint32_t Port, USBPD_FunctionalState State, TCPC_hard_reset HardReset)
{
  uint8_t vbus_level = 0;

  state[Port].Registers.Control.s.u5.POWER_CONTROL = 0;
  if (USBPD_ENABLE == State)
  {
    uint16_t TimeoutSink = 0;

    if (state[Port].PowerRole == USBPD_PORTPOWERROLE_SRC)
    {
      /* Function can be called after sending a hard reset.
         We should not wait for VSafe0V).*/
      if (TCPC_HARD_RESET_NONE == HardReset)
      {
        /* In case of Ellisys test 4.5.4, VBUS is still high.
           Should wait for VSafe0V. */
        uint16_t vbus_voltage = 0;
        /* Wait for VBUS ready */
        fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
        while (vbus_voltage > 800)
        {
          fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
        }
      }

      /* Enable VCONN */
      //state[Port].Registers.Control.s.u5.b5.EN_VCONN    = 1;
      state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 1;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);

      /* Disable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      /* Enable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_SRC_VBUS_DEFAULT;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    }
    else
    {
      /* Disable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      /* Enable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_ENABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    }

    /* Wait for VBUS ready */
    fusb305_tcpc_get_vbus_level(Port, &vbus_level, NULL);
    while (vbus_level != 1)
    {
      /* Add a delay for CC debounce */
      USBPD_TCPCI_Delay(4);

      USBPD_TCPCI_ReadRegister(Port, TCPC_REG_CC_STATUS, (uint8_t*)&state[Port].Registers.Status.u1.CC_STATUS, 1);
      /* Check if CC line has been disconnected */
      /* or if no more VBUS after CC_DEBOUNCE_TIMER x 3 (around 12ms) */
      if (((TYPEC_CC_VOLT_OPEN == state[Port].Registers.Status.u1.b1.CC1_STAT) 
           && (state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT == 0))
          || ((TYPEC_CC_VOLT_OPEN == state[Port].Registers.Status.u1.b1.CC2_STAT) 
              && (state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT == 1))
          || ((state[Port].PowerRole == USBPD_PORTPOWERROLE_SNK) && (TimeoutSink > (1000 / CC_DEBOUNCE_TIMER))))
      {
        /* Disable SOURCE VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
        /* Disable SINK VBUS */
        state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);

        /* Disable VCONN */
        state[Port].Registers.Control.s.u5.b5.EN_VCONN    = 0;
        state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 0;
        state[Port].Registers.Control.s.u5.b5.DIS_VALRM   = 1;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);
        /* Disable BIST mode */
        state[Port].Registers.Control.s.u2.b2.BIST_TMODE = 0;
        USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);
        return USBPD_FAIL;
      }
      TimeoutSink++;
      fusb305_tcpc_get_vbus_level(Port, &vbus_level, NULL);
    }

    /* Enable auto power discharge */
    if (state[Port].PowerRole == USBPD_PORTPOWERROLE_SRC)
    {
      /* Disable BIST mode. Issue if AUTO_DISCH is enabled during BIST tests */
      if (state[Port].Registers.Control.s.u2.b2.BIST_TMODE == 0)
        state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 1;
    }
    else
    {
      /* Disable VCONN */
      state[Port].Registers.Control.s.u5.b5.DIS_VALRM   = 1;
    }
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);
  }
  else
  {
    uint16_t vbus_voltage = 0;
    /* Disable SOURCE VBUS */
    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    /* Disable SINK VBUS */
    state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
    
    /* Wait for VBUS ready 
       In case of 4.5.4, a CC detach is detected but value of VBUS is kept to higher than 3.76V */
    fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
    while ((vbus_level != 0) && (vbus_voltage > 4000)) {
      /* Disable SOURCE VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SRC_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      /* Disable SINK VBUS */
      state[Port].Registers.Command.u.COMMAND = TCPC_REG_COMMAND_DISABLE_SINK_VBUS;
      USBPD_TCPCI_WriteRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.u.COMMAND, 1);
      fusb305_tcpc_get_vbus_level(Port, &vbus_level, &vbus_voltage);
    };

    /* Disable auto VBUS discharge */
    state[Port].Registers.Control.s.u5.b5.EN_VCONN    = 0;
    state[Port].Registers.Control.s.u5.b5.AUTO_DISCH  = 0;
    state[Port].Registers.Control.s.u5.b5.DIS_VALRM   = 1;
    USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_CONTROL, &state[Port].Registers.Control.s.u5.POWER_CONTROL, 1);
  }

  return USBPD_OK;
}

/**
  * @brief  Management of ALERT
  * @param  Port port number value
  * @param  Alert Pointer on ALERT
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_alert(uint32_t Port, uint16_t *Alert)
{
  USBPD_StatusTypeDef usbpd_status = USBPD_OK;
  
  /* Read the Alert register from the TCPC */
  usbpd_status = USBPD_TCPCI_ReadRegister(Port, TCPC_REG_ALERT, (uint8_t*)&state[Port].Registers.Alerts.word[0], 2);

  /* Read the Alert register from the TCPC */
  usbpd_status = USBPD_TCPCI_ReadRegister(Port, TCPC_REG_ALERT_VD, (uint8_t*)&state[Port].Vendor.ALERT_VD, 1);

  *Alert = state[Port].Registers.Alerts.word[0];
#if defined(_TRACE)
  {
    uint8_t tab[15] = {0};
    uint8_t size = sprintf((char*)tab,"Alert=%2x", *Alert);
    FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
  }
#endif /* _TRACE */
  
  /*
  * Clear alert status for everything except RX_STATUS, which should not
  * be cleared until we have successfully retrieved message.
  */
  if (usbpd_status == USBPD_OK)
  {
    uint16_t alert = (TCPC_REG_ALERT_CLEAR_ALL & ~(TCPC_REG_ALERT_RECEIVE_SOP|TCPC_REG_ALERT_TRANSMIT_COMPLETE));
    usbpd_status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT, (uint8_t*)&alert, 2);
    
    /* Keep only the relevant alerts enabled in the alert mask */
    *Alert &= state[Port].Registers.Alerts.word[1];
#if defined(_TRACE)
    {
      uint8_t tab[15] = {0};
      uint8_t size = sprintf((char*)tab,"ValidAlert=%2x", *Alert);
      FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, tab);
    }
#endif /* _TRACE */
  }
  return usbpd_status;
}

/**
  * @brief  Clear ALERT on TCPC device
  * @param  Port  port number value
  * @param  Alert Pointer on ALERT to clear
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_clear_alert(uint32_t Port, uint16_t *Alert)
{
  USBPD_StatusTypeDef usbpd_status = USBPD_OK;
  uint8_t reg = 0xFF;
  usbpd_status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT, (uint8_t*)Alert, 2);

  /* Clear vendor alert */
  usbpd_status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT_VD, &reg, 1);
  
  return usbpd_status;
}

/**
  * @brief  Set BIST data managment
  * @param  Port port number value
  * @param  Enable Enable BIST Carrier mode 2 or not
  * @retval USBPD status
  */
USBPD_StatusTypeDef fusb305_tcpc_set_bist_test_data(uint32_t Port, uint8_t Enable)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  if (Enable)
  {
    state[Port].Registers.Control.s.u2.b2.BIST_TMODE = 1;
    status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);
  }
  else
  {
    /* Disable BIST only if it was enabled */
    if (state[Port].Registers.Control.s.u2.b2.BIST_TMODE == 1)
    {
      state[Port].Registers.Control.s.u2.b2.BIST_TMODE = 0;
      status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_TCPC_CONTROL, &state[Port].Registers.Control.s.u2.TCPC_CONTROL, 1);
    }
  }
  return status;
}

/**
 * @brief  function to set the SinkTxNg
 * @param  PortNum  Number of the port.
 * @retval none.
  */
USBPD_StatusTypeDef fusb305_tcpc_SinkTxNG(uint32_t PortNum)
{
#if 1
  state[PortNum].Registers.Control.s.u3.b3.RP_VAL = TYPEC_RP_VALUE_1P5A;
  return USBPD_TCPCI_WriteRegister(PortNum, TCPC_REG_ROLE_CONTROL, &state[PortNum].Registers.Control.s.u3.ROLE_CONTROL, 1);
#else
  return USBPD_OK;
#endif
}

/**
 * @brief  function to set the SinkTxOK
 * @param  PortNum  Number of the port.
 * @retval none.
  */
USBPD_StatusTypeDef fusb305_tcpc_SinkTxOK(uint32_t PortNum)
{
#if 1
  /* Save the current RP value */
  state[PortNum].RP_Value = (TCPC_RP_Value_TypeDef)state[PortNum].Registers.Control.s.u3.b3.RP_VAL;

  /* Set the new RP value to 3.0A */
  state[PortNum].Registers.Control.s.u3.b3.RP_VAL = TYPEC_RP_VALUE_3P0A;
  return USBPD_TCPCI_WriteRegister(PortNum, TCPC_REG_ROLE_CONTROL, &state[PortNum].Registers.Control.s.u3.ROLE_CONTROL, 1);
#else
  return USBPD_OK;
#endif
}

/**
 * @brief  function to check if SinkTxOK
 * @param  PortNum  Number of the port.
 * @retval USBPD status based on @ref USBPD_StatusTypeDef
  */
USBPD_StatusTypeDef fusb305_tcpc_IfSinkTxOk(uint32_t PortNum)
{
#if 0
  USBPD_StatusTypeDef status = USBPD_FAIL;
  USBPD_TCPCI_ReadRegister(PortNum, TCPC_REG_ROLE_CONTROL, &state[PortNum].Registers.Control.s.u3.ROLE_CONTROL, 1);

  if (TYPEC_RP_VALUE_1P5A == state[PortNum].Registers.Control.s.u3.b3.RP_VAL)
  {
    status = USBPD_OK;
  }
  return status;
#else
  return USBPD_OK;
#endif
}

/**
  * @}
  */

/** @addtogroup FUSB305_TCPC_Private_Functions
 * @{
 */
/**
  * @brief  Initialize all the FUSB305 regisiter
  * @param  Port port number value
  * @retval none
  */
static void InitializeRegisters(uint32_t Port)
{
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_VENDOR_ID, (uint8_t*)&state[Port].Registers.TCPC_Information.word[0], 12);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_ALERT, (uint8_t*)&state[Port].Registers.Alerts.word[0], 4);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS_MASK, &state[Port].Registers.StatusMask.byte[0], 2);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_CONFIG_STANDARD_OUTPUT, &state[Port].Registers.Control.byte[0], 5);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_CC_STATUS, &state[Port].Registers.Status.u1.CC_STATUS, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS, &state[Port].Registers.Status.u2.POWER_STATUS, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_FAULT_STATUS, &state[Port].Registers.Status.u3.FAULT_STATUS, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_COMMAND, &state[Port].Registers.Command.byte, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_DECIVE_CAP1, &state[Port].Registers.Capabilities.byte[0], 6);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_MSG_HEADER_INFO, (uint8_t*)&state[Port].Registers.FrameInfo.u1.MESSAGE_HEADER_INFO, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_RX_DETECT, (uint8_t*)&state[Port].Registers.FrameInfo.u2.RECEIVE_DETECT, 1);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_RX_BYTE_COUNT, &state[Port].Registers.RXFrame.byte[0], 4);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_TRANSMIT, &state[Port].Registers.TXFrame.byte[0], 4);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_VBUS_VOLTAGE, (uint8_t*)&state[Port].Registers.VBUS.word[0], 10);
  USBPD_TCPCI_ReadRegister(Port, TCPC_REG_VCONN_OCP, (uint8_t*)&state[Port].Vendor.byte[0], 21);
}

/**
  * @brief  Initialize all the FUSB305 registers
  * @param  Port       Port number value
  * @param  PowerRole  Power role of the connection
  * @param  State      State of the connection
  * @retval USBPD status
  */
static USBPD_StatusTypeDef tcpc_set_alert_mask(uint32_t Port,  uint8_t PowerRole, USBPD_FunctionalState State)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;

  /* Reset all the masks */
  state[Port].Registers.Alerts.s.u3.ALERTMSKL        = 0;
  state[Port].Registers.Alerts.s.u4.ALERTMSKH        = 0;
  
  if (USBPD_ENABLE == State)
  {
    if (PowerRole == USBPD_PORTPOWERROLE_SRC)
    {
      state[Port].Registers.Alerts.s.u3.b3.M_CCSTAT         = 1;
    }
    else
    {
      state[Port].Registers.Alerts.s.u3.b3.M_CCSTAT         = 1;
      state[Port].Registers.Alerts.s.u3.b3.M_PORT_PWR       = 1;
      state[Port].Registers.Alerts.s.u4.b4.M_VBUS_SNK_DISC  = 1;
    }
    
    state[Port].Registers.Alerts.s.u3.b3.M_TXSUCC         = 1;
    state[Port].Registers.Alerts.s.u3.b3.M_TXFAIL         = 1;
    state[Port].Registers.Alerts.s.u3.b3.M_TXDISC         = 1;
    state[Port].Registers.Alerts.s.u3.b3.M_RXSTAT         = 1;
    state[Port].Registers.Alerts.s.u3.b3.M_RXHRDRST       = 1;
    //state[Port].Registers.Alerts.M_FAULT       = 1;
  }
  else
  {
    /*
    * Create mask of alert events that will cause the TCPC to
    * signal the TCPM via the Alert# gpio line.
    */
    state[Port].Registers.Alerts.s.u3.b3.M_CCSTAT = 1;
  }

  status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ALERT_MASK, (uint8_t*)&state[Port].Registers.Alerts.word[1], 2);
  /* Set the alert mask in TCPC */
  return status;
}

/**
  * @brief  Initialize power status mask
  * @param  Port port number value
  * @retval USBPD status
  */
static USBPD_StatusTypeDef tcpc_init_power_status_mask(uint32_t Port)
{
  USBPD_StatusTypeDef status = USBPD_FAIL;
  
  state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK = TCPC_REG_POWER_STATUS_MASK_VBUS_PRES;
  status = USBPD_TCPCI_WriteRegister(Port, TCPC_REG_POWER_STATUS_MASK , &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  if (status == USBPD_OK)
  {
    status =   USBPD_TCPCI_ReadRegister(Port, TCPC_REG_POWER_STATUS_MASK, &state[Port].Registers.StatusMask.s.u1.POWER_STATUS_MASK, 1);
  }
  /* Set the alert mask in TCPC */
  return status;
}

/**
  * @brief  Set role of each CC pin
  * @param  Port port number value
  * @param  Pull CC pin value
  * @param  State  State of the connection
  * @retval USBPD status
  */
static void tcpc_set_pin_role(uint32_t Port, uint8_t Pull, USBPD_FunctionalState State)
{
#if defined(_TRACE)
    switch (Pull)
    {
      case TYPEC_CC_RA:
        FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, "tcpc_set_pin_role(RA)");
        break;
      case TYPEC_CC_RP:
        FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, "tcpc_set_pin_role(RP)");
        break;
      case TYPEC_CC_RD:
        FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, "tcpc_set_pin_role(RD)");
        break;
      case TYPEC_CC_OPEN:
        FUSB307_DEBUG_TRACE(Port, FUSB307_DEBUG_LEVEL_0, "tcpc_set_pin_role(OPEN)");
        break;
      }
#endif /* _TRACE */
  if (USBPD_ENABLE == State)
  {
#if 0
    if (TYPEC_CC_OPEN == state[Port].VConnPresence)
    {
      /* PD communication set on CC1 */
      state[Port].Registers.Control.s.u3.ROLE_CONTROL = TCPC_REG_ROLE_CONTROL_SET(0, TYPEC_RP_VALUE_3P0A, Pull, Pull);
    }
    else
#endif
    {
      /* Set role depending on polarity */
      if (state[Port].Registers.Control.s.u2.b2.PLUG_ORIENT == 0)
      {
        /* PD communication set on CC1 */
        state[Port].Registers.Control.s.u3.ROLE_CONTROL = TCPC_REG_ROLE_CONTROL_SET(0, TYPEC_RP_VALUE_1P5A, Pull, TYPEC_CC_OPEN);
      }
      else
      {
        /* PD communication set on CC2 */
        state[Port].Registers.Control.s.u3.ROLE_CONTROL = TCPC_REG_ROLE_CONTROL_SET(0, TYPEC_RP_VALUE_1P5A, TYPEC_CC_OPEN, Pull);
      }
    }
  }
  else
  {
    /* Set RP value to 0 to save power */
    state[Port].Registers.Control.s.u3.ROLE_CONTROL = TCPC_REG_ROLE_CONTROL_SET(state[Port].TogglingEnable, TYPEC_RP_VALUE_1P5A, Pull, Pull);
  }
  USBPD_TCPCI_WriteRegister(Port, TCPC_REG_ROLE_CONTROL, &state[Port].Registers.Control.s.u3.ROLE_CONTROL, 1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

