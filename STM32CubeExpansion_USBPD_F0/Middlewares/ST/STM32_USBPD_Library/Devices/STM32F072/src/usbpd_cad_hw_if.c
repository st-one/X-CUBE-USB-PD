/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains CAD interfaces functions.
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
#include "usbpd_def.h"
#include "usbpd_trace.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_hw_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef enum
{
  USBPD_CAD_STATE_RESET           =0,  /*!< USBPD CAD State Reset                              */
  USBPD_CAD_STATE_DETACHED        =1,  /*!< USBPD CAD State No cable detected                  */
  USBPD_CAD_STATE_ATTACHED_WAIT   =2,  /*!< USBPD CAD State Port partner detected              */
  USBPD_CAD_STATE_ATTACHED        =3,  /*!< USBPD CAD State Port partner attached              */
  USBPD_CAD_STATE_EMC             =4,  /*!< USBPD CAD State Electronically Marked Cable detected   */
  USBPD_CAD_STATE_ATTEMC          =5,  /*!< USBPD CAD State Port Partner detected throug EMC   */
  USBPD_CAD_STATE_ACCESSORY       =6,  /*!< USBPD CAD State Accessory detected                 */
  USBPD_CAD_STATE_DEBUG           =7,  /*!< USBPD CAD State Debug detected                     */
  USBPD_CAD_STATE_SWITCH_TO_SRC   =8,  /*!< USBPD CAD State switch to Source                   */
  USBPD_CAD_STATE_SWITCH_TO_SNK   =9,  /*!< USBPD CAD State switch to Sink                     */
  USBPD_CAD_STATE_ATTACHED_LEGACY =10, /*!< USBPD CAD State Port partner attached to legacy cable */
  USPPD_CAD_STATE_UNKNOW          =11  /*!< USBPD CAD State unknow                             */
} USBPD_CAD_STATE;
/**
  * @}
  */

/**
  * @brief USB PD CC lines HW condition
  */
typedef enum
{
  HW_Detachment                         = 0x00,    /*!< Nothing attached                        */
  HW_Attachment                         = 0x01,    /*!< Sink attached                           */
  HW_PwrCable_NoSink_Attachment         = 0x02,    /*!< Powered cable without Sink attached     */
  HW_PwrCable_Sink_Attachment           = 0x03,    /*!< Powered cable with Sink or VCONN-powered Accessory attached   */
  HW_Debug_Attachment                   = 0x04,    /*!< Debug Accessory Mode attached           */
  HW_AudioAdapter_Attachment            = 0x05     /*!< Audio Adapter Accessory Mode attached   */
} CAD_HW_Condition_TypeDef;


/**
  * @brief CAD State value @ref USBPD_DEVICE_CAD_HW_IF
  * @{
  */
typedef struct
{
  USBPD_SettingsTypeDef *settings;
  USBPD_ParamsTypeDef   *params;
  void (*USBPD_CAD_WakeUp)(void);
  uint32_t tToogle_start;

  USBPD_CAD_STATE state: 4;
  CCxPin_TypeDef  cc: 2;
  CAD_HW_Condition_TypeDef    CurrentHWcondition: 3;
  CAD_HW_Condition_TypeDef    OldHWCondtion: 3;
  CAD_SNK_Source_Current_Adv_Typedef SNK_Source_Current_Adv: 2;
  uint32_t reserved: 18;
} CAD_HW_HandleTypeDef;
/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef __DEBUG_CAD
USBPD_CAD_STATE   a_State[USBPD_PORT_COUNT][256];
uint8_t indexState[USBPD_PORT_COUNT];
__IO CAD_HW_Condition_TypeDef    a_HWCondition[USBPD_PORT_COUNT][256];          /*!< USBPD CAD Old HW condition    */
__IO uint8_t indexHWcond[USBPD_PORT_COUNT];
__IO CCxPin_TypeDef a_CAD_CurrentCC[USBPD_PORT_COUNT][256];
uint8_t indexCurrentCC[USBPD_PORT_COUNT];
#endif /* __DEBUG_CAD */

/* Variable containing ADC conversions results */
extern uint32_t             ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];
/* CAD_tCCDebounce             Time a port shall wait before it can determine it is attached*/
uint32_t CAD_tCCDebounce_start[USBPD_PORT_COUNT], CAD_tCCDebounce[USBPD_PORT_COUNT];
/* CAD_tPDDebounce_start       (1) CAD_tPDDebounce counting starts, (0) CAD_tPDDebounce counting reset */
uint32_t CAD_tPDDebounce_start[USBPD_PORT_COUNT], CAD_tPDDebounce[USBPD_PORT_COUNT];
/* CAD_tPDDebounce             Time a port shall wait before it can determine it is detached*/
uint8_t CAD_tPDDebounce_flag[USBPD_PORT_COUNT];

/* handle to manage the detection state machine */
static CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT];

extern USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_DEVICE_CAD_HW_IF_Private_Functions USBPD DEVICE_CAD HW IF Private Functions
  * @{
  */
static inline void    SignalDetachment(uint8_t PortNum);
static inline void    SignalSwitchRole(uint8_t PortNum, USBPD_CAD_STATE State);
static uint8_t        Check_VBus(uint8_t PortNum);
static CCxPin_TypeDef Check_HW(uint8_t PortNum);
extern  uint8_t       USBPDM1_IsAnalog(uint8_t PortNum, CCxPin_TypeDef cc);
/**
  * @}
  */

/** @defgroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions USBPD DEVICE_CAD HW IF Exported Functions
  * @{
  */

/**
  * @brief  CAD initialization function
  * @param  PortNum       port
  * @param  Settings      Pointer on PD settings based on @ref USBPD_SettingsTypeDef
  * @param  Params        Pointer on PD params based on @ref USBPD_ParamsTypeDef
  * @param  WakeUp        Wake-up callback function used for waking up CAD (not used by STM32F072 device)
  * @retval None
  */
void CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *Settings, USBPD_ParamsTypeDef *Params,  void (*WakeUp)(void))
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];

  _handle->params = Params;
  Ports[PortNum].params = Params;
  _handle->settings = Settings;
  _handle->state = USBPD_CAD_STATE_RESET;
  _handle->cc = CCNONE;
  _handle->CurrentHWcondition = _handle->OldHWCondtion = HW_Detachment;
  _handle->SNK_Source_Current_Adv = vRd_Undefined;

  if (_handle->params->PE_PowerRole == USBPD_PORTPOWERROLE_SRC)
  {
    USBPDM1_DeAssertRd(PortNum);
    USBPDM1_AssertRp(PortNum);
  }
  else
  {
    USBPDM1_DeAssertRp(PortNum);
    USBPDM1_AssertRd(PortNum);
  }
}

/**
  * @brief  CAD State machine
  * @param  PortNum Port
  * @param  Event   Pointer on CAD event based on @ref USBPD_CAD_EVENT
  * @param  CCXX    Pointer on CC Pin based on @ref CCxPin_TypeDef
  * @retval Timeout value
  */
uint32_t CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  uint32_t _timing;
  
  /* set by default event to none */
  *Event = USBPD_CAD_EVENT_NONE;
  
  if(_handle->params->PE_SwapOngoing != 0)
  {
    return 2;
  }


  do{
    
    _timing = 2;
  
  /*Check CAD STATE*/
  switch (_handle->state)
  {
      /*CAD STATE DETACHED*/
    case USBPD_CAD_STATE_RESET:
    case USBPD_CAD_STATE_SWITCH_TO_SRC:
    case USBPD_CAD_STATE_SWITCH_TO_SNK:
    {
      if (_handle->settings->CAD_RoleToggle == USBPD_TRUE)
      {
        _handle->tToogle_start = HAL_GetTick();
        if (USBPD_CAD_STATE_SWITCH_TO_SRC == _handle->state)
        {
          USBPDM1_DeAssertRd(PortNum);
          USBPDM1_AssertRp(PortNum);
          _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
        }
        if (USBPD_CAD_STATE_SWITCH_TO_SNK == _handle->state)
        {
          USBPDM1_DeAssertRp(PortNum);
          USBPDM1_AssertRd(PortNum);
          _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
        }
      }
      _handle->state = USBPD_CAD_STATE_DETACHED;
    }
    break;

    case USBPD_CAD_STATE_DETACHED:
    {
      /* Check CCx HW condition*/
      _handle->cc = Check_HW(PortNum);
      /* Change the status on the basis of the HW event given by Check_HW() */
      if (_handle->CurrentHWcondition != HW_Detachment)
      {
        _handle->OldHWCondtion = _handle->CurrentHWcondition;

        if (_handle->CurrentHWcondition == HW_PwrCable_NoSink_Attachment)
        {
          _handle->state = USBPD_CAD_STATE_EMC;
        }
        else
        {
          /* Check if legacy cable have detected (refer to Type-C specification
             'Legacy Cable Assemblies' */
          if ((USBPD_PORTPOWERROLE_SNK == _handle->params->PE_PowerRole)
              && Check_VBus(PortNum)
              && (vRd_USB == _handle->SNK_Source_Current_Adv))
          {
            /* VBUS is already present. Port Partner does not support PD protocol (USB Type A/B)
               Only Default USB Type-C Current Rp resistor (56 k) has been detected */
            _handle->state = USBPD_CAD_STATE_ATTACHED_LEGACY;
            *Event = USBPD_CAD_EVENT_LEGACY;
          }
          else
          {
            _handle->state = USBPD_CAD_STATE_ATTACHED_WAIT;
            /* Get the time of this event */
            CAD_tCCDebounce_start[PortNum] = HAL_GetTick();
          }
          break;
        }
      }
      else
      {
        _handle->state = USBPD_CAD_STATE_DETACHED;

        /* check if toggle timer ended */
        if (_handle->settings->CAD_RoleToggle == USBPD_TRUE)
        {
          if ((USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole)
              && ((HAL_GetTick() - _handle->tToogle_start) > _handle->settings->CAD_SRCToogleTime))
          {
            _handle->tToogle_start = HAL_GetTick();
            _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
            USBPDM1_DeAssertRp(PortNum);
            USBPDM1_AssertRd(PortNum);
          }
          if ((USBPD_PORTPOWERROLE_SNK == _handle->params->PE_PowerRole)
              && ((HAL_GetTick() - _handle->tToogle_start) > _handle->settings->CAD_SNKToogleTime))
          {
            _handle->tToogle_start = HAL_GetTick();
            _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
            USBPDM1_DeAssertRd(PortNum);
            USBPDM1_AssertRp(PortNum);
          }
        }
      }
    }
    break;

    /*CAD STATE ATTACHED WAIT*/
    case USBPD_CAD_STATE_ATTACHED_WAIT:
    {
      /* Evaluate elapsed time in Attach_Wait state */
      CAD_tCCDebounce[PortNum] = HAL_GetTick() - CAD_tCCDebounce_start[PortNum];
      /* Check CCx HW condition*/
      _handle->cc = Check_HW(PortNum);

      if (!((_handle->CurrentHWcondition == HW_Detachment) || (_handle->CurrentHWcondition == HW_PwrCable_NoSink_Attachment))
          && (_handle->CurrentHWcondition == _handle->OldHWCondtion))
      {
        if ((ADCxConvertedValues[VBUS_INDEX(PortNum)] > 300)
            && (USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole))
        {
          _handle->state = USBPD_CAD_STATE_ATTACHED_WAIT;
          /* Get the time of this event */
          CAD_tCCDebounce_start[PortNum] = HAL_GetTick();
          break;
        }

        /* Check tCCDebounce */
        if (CAD_tCCDebounce[PortNum] > CAD_tCCDebounce_threshold)
        {
          /* if tCCDebounce has expired state must be changed*/
          if (USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole)
          {
            switch (_handle->CurrentHWcondition)
            {
              case HW_Attachment:
                _handle->state = USBPD_CAD_STATE_ATTACHED;
                *Event = USBPD_CAD_EVENT_ATTACHED;
                USBPDM1_Set_CC(PortNum, _handle->cc);
                USBPDM1_RX_EnableInterrupt(PortNum);
                break;

              case HW_PwrCable_NoSink_Attachment:
                _handle->state = USBPD_CAD_STATE_EMC;
                *Event = USBPD_CAD_EVENT_EMC;
                USBPDM1_Set_CC(PortNum, _handle->cc);
                USBPDM1_RX_EnableInterrupt(PortNum);
                break;

              case HW_PwrCable_Sink_Attachment:
                _handle->state = USBPD_CAD_STATE_ATTEMC;
                *Event = USBPD_CAD_EVENT_ATTEMC;
                USBPDM1_Set_CC(PortNum, _handle->cc);
                USBPDM1_RX_EnableInterrupt(PortNum);
                break;

              case HW_Debug_Attachment:
                _handle->state = USBPD_CAD_STATE_DEBUG;
                *Event = USBPD_CAD_EVENT_DEBUG;
                break;

              case HW_AudioAdapter_Attachment:
                HW_SignalDetachment(PortNum, CC2);
                HW_SignalDetachment(PortNum, CC1);
                _handle->state = USBPD_CAD_STATE_ACCESSORY;
                *Event = USBPD_CAD_EVENT_ACCESSORY;
                break;

              case HW_Detachment:
              default:
                break;
            } /* end of switch */
            *CCXX = _handle->cc;
          }
          else /* Check state transition for SNK role */
          {
            if (Check_VBus(PortNum)) /* Check if Vbus is on */
            {
              _handle->state = USBPD_CAD_STATE_ATTACHED;
              USBPDM1_Set_CC(PortNum, _handle->cc);
              USBPDM1_RX_EnableInterrupt(PortNum);
              *CCXX = _handle->cc;
              *Event = USBPD_CAD_EVENT_ATTACHED;
            }
          }
        }
        /* reset the flag for CAD_tPDDebounce */
        CAD_tPDDebounce_flag[PortNum] = 0;
      }
      else /* CAD_HW_Condition[PortNum] = HW_Detachment */
      {
        /*Check tPDDebounce*/
        if (USBPD_PORTPOWERROLE_SNK == _handle->params->PE_PowerRole)
        {

          /* start counting of CAD_tPDDebounce */
          if (CAD_tPDDebounce_flag[PortNum] == 0)
          {
            CAD_tPDDebounce_start[PortNum] = HAL_GetTick();
            CAD_tPDDebounce_flag[PortNum] = 1;
          }
          else /* CAD_tPDDebounce already running */
          {
            /* evaluate CAD_tPDDebounce */
            CAD_tPDDebounce[PortNum] = HAL_GetTick() - CAD_tPDDebounce_start[PortNum];
            if (CAD_tPDDebounce[PortNum] > CAD_tPDDebounce_threshold)
            {
              CAD_tPDDebounce_flag[PortNum] = 0;
              SignalSwitchRole(PortNum, USBPD_CAD_STATE_SWITCH_TO_SRC);
              _timing = 0;
            }
          }
        }
        else /* (hcad->PortPowerRole != USBPD_PORTPOWERROLE_SNK)*/
        {
          SignalSwitchRole(PortNum, USBPD_CAD_STATE_SWITCH_TO_SNK);
        }
      }
    }
    break;

    /*CAD STATE ATTACHED*/
    case USBPD_CAD_STATE_ATTACHED:
    {
      if (USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole)
      {
        /* Check CCx HW condition*/
        _handle->cc = Check_HW(PortNum);

        if (_handle->CurrentHWcondition == HW_Detachment)
        {
          SignalDetachment(PortNum);
          *Event = USBPD_CAD_EVENT_DETACHED;
          *CCXX = _handle->cc;
        }
      }
      else /* hcad->PortPowerRole != USBPD_PORTPOWERROLE_SRC) */
      {
        if (Check_VBus(PortNum) == 0) /* Check if Vbus is off */
        {
          SignalDetachment(PortNum);
          *Event = USBPD_CAD_EVENT_DETACHED;
          _handle->state = USBPD_CAD_STATE_DETACHED;
          *CCXX = _handle->cc;
        }
      }
    }
    break;

    /*CAD STATE ATTACHED TO LEGACY CABLES */
    case USBPD_CAD_STATE_ATTACHED_LEGACY:
    {
      if (Check_VBus(PortNum) == 0) /* Check if Vbus is off */
      {
        SignalDetachment(PortNum);
        *Event = USBPD_CAD_EVENT_DETACHED;
        *CCXX = _handle->cc;
      }
    }
    break;
    /*CAD ELECTRONIC CABLE ATTACHED*/
    case USBPD_CAD_STATE_EMC:
    {
      /* Check CCx HW condition*/
      _handle->cc = Check_HW(PortNum);

      switch (_handle->CurrentHWcondition)
      {
        case HW_Detachment:
          SignalDetachment(PortNum);
          /* Forward event information to upper layer */
          *Event = USBPD_CAD_EVENT_DETACHED;
          *CCXX = _handle->cc;
          break;
        case HW_Attachment:
          /* Sink has been attached */
          _handle->OldHWCondtion = _handle->CurrentHWcondition;
          _handle->state = USBPD_CAD_STATE_ATTACHED_WAIT;
          /* Get the time of this event */
          CAD_tCCDebounce_start[PortNum] = HAL_GetTick();
          break;
        case HW_PwrCable_Sink_Attachment:
          /* Sink has been attached */
          _handle->OldHWCondtion = _handle->CurrentHWcondition;
          _handle->state = USBPD_CAD_STATE_ATTACHED_WAIT;
          /* Get the time of this event */
          CAD_tCCDebounce_start[PortNum] = HAL_GetTick();
          break;
        case HW_PwrCable_NoSink_Attachment:
          /* continue the toggle operation */
          /* check if toggle timer ended */
          if (_handle->settings->CAD_RoleToggle == USBPD_TRUE)
          {
            if ((USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole)
                && ((HAL_GetTick() - _handle->tToogle_start) > _handle->settings->CAD_SRCToogleTime))
            {
              _handle->tToogle_start = HAL_GetTick();
              _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
              USBPDM1_DeAssertRp(PortNum);
              USBPDM1_AssertRd(PortNum);
              _handle->state = USBPD_CAD_STATE_DETACHED;
            }
            if ((USBPD_PORTPOWERROLE_SNK == _handle->params->PE_PowerRole)
                && ((HAL_GetTick() - _handle->tToogle_start) > _handle->settings->CAD_SNKToogleTime))
            {
              _handle->tToogle_start = HAL_GetTick();
              _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
              USBPDM1_DeAssertRd(PortNum);
              USBPDM1_AssertRp(PortNum);
            }
          }
        default:
          break;
      }
    }
    break;

    /*CAD electronic cable with Sink ATTACHED*/
    case USBPD_CAD_STATE_ATTEMC:
    {
      if(USBPD_PORTPOWERROLE_SNK == _handle->params->PE_PowerRole)
      {
        _handle->state = USBPD_CAD_STATE_ATTACHED;
      }
      else
      {
        CCxPin_TypeDef _ccxx;

        /* Check CCx HW condition*/
        _ccxx = Check_HW(PortNum);

        if ((_handle->CurrentHWcondition == HW_Detachment)
            || (_handle->CurrentHWcondition == HW_PwrCable_NoSink_Attachment))
        {
          *CCXX = _handle->cc = _ccxx;
          if (_handle->CurrentHWcondition == HW_PwrCable_NoSink_Attachment)
          {
            SignalSwitchRole(PortNum, USBPD_CAD_STATE_EMC);
            *Event = USBPD_CAD_EVENT_EMC;
          }
          else
          {
            SignalSwitchRole(PortNum, USBPD_CAD_STATE_SWITCH_TO_SNK);
            *Event = USBPD_CAD_EVENT_DETACHED;
          }
        }
      }
    }
    break;

    /*CAD STATE AUDIO ACCESSORY ATTACHED*/
    case USBPD_CAD_STATE_ACCESSORY:
    {
      /* Check CCx HW condition*/
      _handle->cc = Check_HW(PortNum);

      switch (_handle->CurrentHWcondition)
      {
        case HW_PwrCable_NoSink_Attachment:
        case HW_Detachment:
          SignalSwitchRole(PortNum, USBPD_CAD_STATE_SWITCH_TO_SNK);
          *CCXX = _handle->cc;
          *Event = USBPD_CAD_EVENT_DETACHED;
          break;
        case HW_AudioAdapter_Attachment:
          break;
        default :
          break;
      }
    }
    break;

    /* CAD STATE DEBUG ACCESSORY MODE ATTACHED */
    case USBPD_CAD_STATE_DEBUG:
    {
      /* Check CCx HW condition*/
      _handle->cc = Check_HW(PortNum);

      if (_handle->CurrentHWcondition != HW_Debug_Attachment)
      {
        SignalDetachment(PortNum);
        *CCXX = _handle->cc;
        *Event = USBPD_CAD_EVENT_DETACHED;
      }
    }
    break;

    default :
      break;
  }

#ifdef __DEBUG_CAD
  if (_handle->state != a_State[PortNum][indexState[PortNum] - 1])
  {
    a_State[PortNum][indexState[PortNum]++] = _handle->state;
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, _handle->state, NULL, 0);
#endif    
  }
#endif /* __DEBUG_CAD */
  
  } while(_timing == 0);
    
  return 2;
}

/**
  * @brief  Return the RP value detected on port partner
  * @param  PortNum Port
  * @retval @ref CAD_SNK_Source_Current_Adv_Typedef
  */
CAD_SNK_Source_Current_Adv_Typedef CAD_GetRPValue(uint8_t PortNum)
{
  uint32_t CC1_value = 0, CC2_value = 0;
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  CAD_SNK_Source_Current_Adv_Typedef old_rp_value;

  /* Save the old RP value */
  old_rp_value = _handle->SNK_Source_Current_Adv;

  /* Check for all possible voltages from the highest to the lowest */

  /* CC2 is the active line?*/
  if (CC1 == _handle->cc)
  {
    /* Read the CC lines voltages */
    CC1_value = ADCxConvertedValues[CC1_INDEX(PortNum)];
    /* 1230 mV => ADC 1526 */
    if (CC1_value > CAD_threshold_SNK_vRd_3_0A)
    {
      _handle->SNK_Source_Current_Adv = vRd_3_0A;
    }
    /* 660 mV => ADC 745 */
    else if (CC1_value > CAD_threshold_SNK_vRd_1_5A)
    {
      _handle->SNK_Source_Current_Adv = vRd_1_5A;
    }
    else
    {
      _handle->SNK_Source_Current_Adv = old_rp_value;
    }
  }
  /* CC2 is the active line?*/
  else
  {
    /* Read the CC lines voltages */
    CC2_value = ADCxConvertedValues[CC2_INDEX(PortNum)];
    /* 1230 mV => ADC 1526 */
    if (CC2_value > CAD_threshold_SNK_vRd_3_0A)
    {
      _handle->SNK_Source_Current_Adv = vRd_3_0A;
    }
    /* 660 mV => ADC 745 */
    else if (CC2_value > CAD_threshold_SNK_vRd_1_5A)
    {
      _handle->SNK_Source_Current_Adv = vRd_1_5A;
    }
    else
    {
      _handle->SNK_Source_Current_Adv = old_rp_value;
    }
  }

  return _handle->SNK_Source_Current_Adv;
}
/**
  * @}
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF_Private_Functions
  * @{
  */

/**
  * @brief  Check if VBus is present or not
  * @param  PortNum  port
  * @retval Return 1 is VBUS is present (0 otherwise)
  */
static uint8_t Check_VBus(uint8_t PortNum)
{
  return (ADCxConvertedValues[VBUS_INDEX(PortNum)] > CAD_threshold_VBus) ? 1 : 0;
}

/**
  * @brief  Check CCx HW condition
  * @param  PortNum port
  * @retval CC pin line based on @ref CCxPin_TypeDef
  */
static CCxPin_TypeDef Check_HW(uint8_t PortNum)
{
  uint32_t CC1_value = 0, CC2_value = 0;
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  /*  return value */

  if ((USBPD_FALSE == USBPDM1_IsAnalog(PortNum, CC1)) || (USBPD_FALSE == USBPDM1_IsAnalog(PortNum, CC2)))
  {
    /* Pin is used in TX or RX so not possible to determine which kind if device is attached */
    return _handle->cc;
  }

  /* Read the CC lines voltages */
  CC1_value = ADCxConvertedValues[CC1_INDEX(PortNum)];
  CC2_value = ADCxConvertedValues[CC2_INDEX(PortNum)];

  CCxPin_TypeDef _CCx = CCNONE;

  switch (_handle->params->PE_PowerRole)
  {
      /*Power Role SRC*/
    case USBPD_PORTPOWERROLE_SRC:
    {
      /*******************************************
         Check cable or Sink was already attached
         If yes, check only the current CC line
       *******************************************/

      /* CAD is already attached?*/
      if ((USBPD_CAD_STATE_ATTACHED == _handle->state) || (USBPD_CAD_STATE_ATTEMC == _handle->state))
      {
        /* CC1 is the active line? */
        if (CC1 == _handle->cc)
        {
          /* CC1 is opened? */
          if (CC1_value >= CAD_threshold_SRC_vRd)
          {
            /* CC1 is opened */
            /* Check if cable is still connected */
            if ((HW_PwrCable_Sink_Attachment == _handle->CurrentHWcondition)
                && (CC2_value < CAD_threshold_SRC_vRa))
            {
              _CCx = CC1;
              _handle->CurrentHWcondition = HW_PwrCable_NoSink_Attachment;
            }
            else
            {
              _CCx = CCNONE;
              _handle->CurrentHWcondition = HW_Detachment;
            }
          }
          else
          {
            /* Rd present on CC1 */
            _CCx = CC1;
          }
          break;
        }

        /* CC2 is the active line?*/
        if (CC2 == _handle->cc)
        {
          if (CC2_value >= CAD_threshold_SRC_vRd)
          {
            /* CC2 is opened */
            /* Check if cable is still connected */
            if ((HW_PwrCable_Sink_Attachment == _handle->CurrentHWcondition)
                && (CC1_value < CAD_threshold_SRC_vRa))
            {
              _CCx = CC2;
              _handle->CurrentHWcondition = HW_PwrCable_NoSink_Attachment;
            }
            else
            {
              _CCx = CCNONE;
              _handle->CurrentHWcondition = HW_Detachment;
            }
          }
          else
          {
            /* Rd present on CC2 */
            _CCx = CC2;
          }
          break;
        }
      }

      /*******************************************
         Cable or Sink was not already attached
       *******************************************/

      /* vRa on CC1 */
      if (CC1_value < CAD_threshold_SRC_vRa)
      {
        if (CC2_value < CAD_threshold_SRC_vRa) /* vRa on CC2 */
        {
          _handle->CurrentHWcondition = HW_AudioAdapter_Attachment;
          _CCx = CCNONE;
          break;
        }
        else
        {
          if (CC2_value < CAD_threshold_SRC_vRd) /* vRd on CC2 */
          {
            _handle->CurrentHWcondition = HW_PwrCable_Sink_Attachment;
            _CCx = CC2;
            break;
          }
          else   /* CC2 open */
          {
            _handle->CurrentHWcondition = HW_PwrCable_NoSink_Attachment;
            _CCx = CC2;
            break;
          }
        }
      }
      /* vRd on CC1 */
      if (CC1_value < CAD_threshold_SRC_vRd)
      {
        if (CC2_value < CAD_threshold_SRC_vRa) /* vRa on CC2 */
        {
          _handle->CurrentHWcondition = HW_PwrCable_Sink_Attachment;
          _CCx = CC1;
          break;
        }
        else
        {
          if (CC2_value < CAD_threshold_SRC_vRd) /* vRd on CC2 */
          {
            _handle->CurrentHWcondition = HW_Debug_Attachment;
            _CCx = CCNONE;
            break;
          }
          else   /* CC2 open */
          {
            _handle->CurrentHWcondition = HW_Attachment;
            _CCx = CC1;
            break;
          }
        }
      }
      /* CC1 open */
      if (CC2_value < CAD_threshold_SRC_vRa) /* vRa on CC2 */
      {
        _handle->CurrentHWcondition = HW_PwrCable_NoSink_Attachment;
        _CCx = CC1;
        break;
      }
      if (CC2_value < CAD_threshold_SRC_vRd) /* vRd on CC2 */
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _CCx = CC2;
        break;
      }
      /* CC2 open */
      _handle->CurrentHWcondition = HW_Detachment;
      _CCx = CCNONE;
      break;
    }

    /*Power Role SNK*/
    case USBPD_PORTPOWERROLE_SNK:
    {
      /* Check for all possible voltages from the highest to the lowest */

      /* 1230 mV => ADC 1526 */
      if (CC1_value > CAD_threshold_SNK_vRd_3_0A)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_3_0A;
        _CCx = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_3_0A)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_3_0A;
        _CCx = CC2;
        break;
      }
      /* 660 mV => ADC 745 */
      if (CC1_value > CAD_threshold_SNK_vRd_1_5A)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_1_5A;
        _CCx = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_1_5A)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_1_5A;
        _CCx = CC2;
        break;
      }
      /* 200 mV => ADC 248 */
      if (CC1_value > CAD_threshold_SNK_vRd_USB)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_USB;
        _CCx = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_USB)
      {
        _handle->CurrentHWcondition = HW_Attachment;
        _handle->SNK_Source_Current_Adv = vRd_USB;
        _CCx = CC2;
        break;
      }
      /* Detached */
      _handle->CurrentHWcondition = HW_Detachment;
      _handle->SNK_Source_Current_Adv = vRd_Undefined;
      _CCx = CCNONE;
      break;
    }

    default :
      break;
  }

#ifdef __DEBUG_CAD
  if (_handle->CurrentHWcondition != a_HWCondition[PortNum][indexHWcond[PortNum] - 1])
  {
    a_HWCondition[PortNum][indexHWcond[PortNum]++] = _handle->CurrentHWcondition;
  }
  if (_handle->cc != a_CAD_CurrentCC[PortNum][indexCurrentCC[PortNum] - 1])
  {
    a_CAD_CurrentCC[PortNum][indexCurrentCC[PortNum]++] = _handle->cc;
  }
#endif /* __DEBUG_CAD */
  return _CCx;
}

static inline void SignalDetachment(uint8_t PortNum)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  HW_SignalDetachment(PortNum, _handle->cc);
  _handle->cc = CCNONE;

  _handle->state =   USBPD_CAD_STATE_RESET;
  if (USBPD_TRUE == _handle->settings->CAD_RoleToggle)
  {
    if (USBPD_PORTPOWERROLE_SRC == _handle->params->PE_PowerRole)
    {
      _handle->state = USBPD_CAD_STATE_SWITCH_TO_SNK;
    }
    else
    {
      _handle->state = USBPD_CAD_STATE_SWITCH_TO_SRC;
    }
  }
}

static inline void SignalSwitchRole(uint8_t PortNum, USBPD_CAD_STATE State)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  HW_SignalDetachment(PortNum, _handle->cc);
  _handle->cc = CCNONE;
  _handle->state = State;
}

/**
  * @brief  CA enter in error recovery mode
  * @param  PortNum port index
  * @retval None
  */
void CAD_Enter_ErrorRecovery(uint8_t PortNum)
{
  // remove resistor terminaison
  // switch CAD_StateMachine to Error Recovery state
  // wakeup CAD task
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

