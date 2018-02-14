/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface cad functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
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
#include "usbpd_cad.h"
#include "usbpd_pe.h"
#ifdef _TRACE
#include "usbpd_trace.h" 
#endif /* _TRACE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define _ELLISYS_4_8_1
#if defined(_TRACE)
#define __DEBUG_CAD
#endif /*_TRACE*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//#define __DEBUG_CAD
#ifdef __DEBUG_CAD
__IO CAD_HW_Condition_TypeDef    a_initialHWCondition[256];             /*!< USBPD CAD HW condition        */
__IO CAD_HW_Condition_TypeDef    a_HWCondition[256];          /*!< USBPD CAD Old HW condition        */
__IO uint8_t indexHWcond2 = 0;
__IO uint8_t indexHWcond1 = 0;
__IO uint32_t tab[256]; 
uint8_t index = 0;
#endif

/* Variable containing ADC conversions results */
extern uint32_t             ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];
/* CAD_tCCDebounce             Time a port shall wait before it can determine it is attached*/ 
uint32_t CAD_tCCDebounce_start[2] = {0, 0}, CAD_tCCDebounce[2] = {0, 0};
/* CAD_tPDDebounce_start       (1) CAD_tPDDebounce counting starts, (0) CAD_tPDDebounce counting reset */
uint32_t CAD_tPDDebounce_start[2] = {0, 0}, CAD_tPDDebounce[2] = {0, 0};
/* CAD_tPDDebounce             Time a port shall wait before it can determine it is detached*/
uint8_t CAD_tPDDebounce_flag[2] = {0, 0}; //, CAD_Is_SetAWD[2] = {0, 0};
/**/
static CAD_HW_Condition_TypeDef CAD_HW_Condition[USBPD_PORT_COUNT]; 
/**/
static CAD_SNK_Source_Current_Adv_Typedef CAD_SNK_Source_Current_Adv[USBPD_PORT_COUNT]; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static inline void SignalDetachment(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum);
static inline void SignalSwitchRole(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum, USBPD_CAD_STATE State);

/**
* @brief  Check if VBus is present or not
* @param  PortNum  port
*/
uint8_t CAD_Check_VBus(uint8_t PortNum)
{
  return ( ADCxConvertedValues[VBUS_INDEX(PortNum)] > CAD_threshold_VBus ) ? 1 : 0;
}

/**
* @brief  Check CCx HW condition 
* @param  PortNum                  port
* @param  CAD_HW_Condition:          CCx HW condition //see enum
* @param  CAD_SNK_Source_Current_Adv:  Sink CC pins Multiple Source Current Advertisements  //see enum
* @param  PortPowerRole:          Port Power Role   
*/
CCxPin_TypeDef CAD_Check_HW(uint8_t PortNum, CAD_HW_Condition_TypeDef *CAD_HW_Condition, CAD_SNK_Source_Current_Adv_Typedef *CAD_SNK_Source_Current_Adv, USBPD_PortPowerRole_TypeDef PortPowerRole)
{
  // CC voltages
  uint32_t CC1_value=0, CC2_value=0;

  // return value
  CCxPin_TypeDef CCout = CCNONE;
  
  /* Read the CC lines voltages */
  CC1_value = ADCxConvertedValues[CC1_INDEX(PortNum)];
  CC2_value = ADCxConvertedValues[CC2_INDEX(PortNum)];
  
  switch(PortPowerRole)
  {
  /*Power Role SRC*/  
  case USBPD_PORTPOWERROLE_SRC:
    {
      //vRa on CC1   
      if (CC1_value < CAD_threshold_SRC_vRa) 
      {
        if (CC2_value < CAD_threshold_SRC_vRa) //vRa on CC2
        {
          *CAD_HW_Condition = HW_AudioAdapter_Attachment; 
          CCout = CCNONE;
          break;
        }
        else
        {
          if (CC2_value < CAD_threshold_SRC_vRd) //vRd on CC2
          {
            *CAD_HW_Condition = HW_PwrCable_Sink_Attachment; 
            CCout = CC2; 
            break;
          }
          else   //CC2 open
          {
            *CAD_HW_Condition = HW_PwrCable_NoSink_Attachment; 
            CCout = CC2; 
            break;
          }
        }
      }
      //vRd on CC1
      if (CC1_value < CAD_threshold_SRC_vRd)
      {
        if (CC2_value < CAD_threshold_SRC_vRa) //vRa on CC2
        {
          *CAD_HW_Condition = HW_PwrCable_Sink_Attachment; 
          CCout = CC1; 
          break;
        }
        else
        {
          if (CC2_value < CAD_threshold_SRC_vRd) //vRd on CC2
          {
            *CAD_HW_Condition = HW_Debug_Attachment; 
            CCout = CCNONE; 
            break;
          }
          else   //CC2 open
          {
            *CAD_HW_Condition = HW_Attachment; 
            CCout = CC1;
            break;
          }
        }
      }
      //CC1 open
      if (CC2_value < CAD_threshold_SRC_vRa) //vRa on CC2
      {
        *CAD_HW_Condition = HW_PwrCable_NoSink_Attachment; 
        CCout = CC1; 
        break;
      }
        if (CC2_value < CAD_threshold_SRC_vRd) //vRd on CC2
        {
          *CAD_HW_Condition = HW_Attachment; 
          CCout = CC2;  
          break;
        }
      //CC2 open
          *CAD_HW_Condition = HW_Detachment; 
          CCout = CCNONE;
          break;
        }

  /*Power Role SNK*/  
  case USBPD_PORTPOWERROLE_SNK:
    {
      /* Check for all possible voltages from the highest to the lowest */
      
      //1230 mV => ADC 1526
      if (CC1_value > CAD_threshold_SNK_vRd_3_0A)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_3_0A;
        CCout = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_3_0A)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_3_0A;        
        CCout = CC2;
        break;
      }
      //660 mV => ADC 745
      if (CC1_value > CAD_threshold_SNK_vRd_1_5A)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_1_5A;
        CCout = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_1_5A)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_1_5A;        
        CCout = CC2;
        break;
      }
      //200 mV => ADC 248
      if (CC1_value > CAD_threshold_SNK_vRd_USB)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_USB;
        CCout = CC1;
        break;
      }
      if (CC2_value > CAD_threshold_SNK_vRd_USB)
      {
        *CAD_HW_Condition = HW_Attachment;
        *CAD_SNK_Source_Current_Adv = vRd_USB;        
        CCout = CC2;
        break;
      }    
      /* Detached */
      *CAD_HW_Condition = HW_Detachment;
      *CAD_SNK_Source_Current_Adv = vRd_Undefined;     
      CCout = CCNONE;    
      break;   
    }

  default :
    break;
  }
  
  return CCout;
}

void CAD_StateMachine(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum)
{
  /* CC variables */
  CCxPin_TypeDef CCXX;

  /*Check CAD STATE*/
  switch(hcad->state)
  {
    /*CAD STATE DETACHED*/  
  case USBPD_CAD_STATE_RESET:
  case USBPD_CAD_STATE_DETACHED:
  case USBPD_CAD_STATE_SWITCH_TO_SRC:    
  case USBPD_CAD_STATE_SWITCH_TO_SNK:    
    /* Check CCx HW condition*/   
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));          
    /* Change the status on the basis of the HW event given by CAD_Check_HW() */
#ifdef _ELLISYS_4_8_1
    if ((CAD_HW_Condition[PortNum] != HW_Detachment))
#else
    if ((CAD_HW_Condition[PortNum] != HW_Detachment) && (CAD_HW_Condition[PortNum] != HW_AudioAdapter_Attachment))
#endif
    {
      hcad->cc    = CCXX;
      hcad->OldHWCondtion = CAD_HW_Condition[PortNum];
#ifdef __DEBUG_CAD
      a_initialHWCondition[indexHWcond1++] = hcad->OldHWCondtion;
      tab[index++] = ADCxConvertedValues[CC1_INDEX(PortNum)];
      tab[index++] = ADCxConvertedValues[CC2_INDEX(PortNum)];
#endif      
      if(CAD_HW_Condition[PortNum] == HW_PwrCable_NoSink_Attachment)
      {
        hcad->state = USBPD_CAD_STATE_EMC;
      }
      else
      {
        hcad->state = USBPD_CAD_STATE_ATTACHED_WAIT;
        /* Get the time of this event */
        CAD_tCCDebounce_start[PortNum] = HAL_GetTick();
        break;
      }
    }
    else
    {
      hcad->state = USBPD_CAD_STATE_DETACHED;
    }
    break;
    
    /*CAD STATE ATTACHED WAIT*/   
  case USBPD_CAD_STATE_ATTACHED_WAIT:
    
    /* Evaluate elapsed time in Attach_Wait state */
    CAD_tCCDebounce[PortNum] = HAL_GetTick() - CAD_tCCDebounce_start[PortNum];
    /* Check CCx HW condition*/  
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum)); 

    if ((CAD_HW_Condition[PortNum] != HW_Detachment) 
      && (CAD_HW_Condition[PortNum] == hcad->OldHWCondtion))
    {
#if defined(_OPTIM_CONSO)
      if((ADCxConvertedValues[VBUS_INDEX(PortNum)] > 500) && (USBPD_PORTPOWERROLE_SRC == PE_GetPowerRole(PortNum)))
#else
      if((ADCxConvertedValues[VBUS_INDEX(PortNum)] > 200) && (USBPD_PORTPOWERROLE_SRC == PE_GetPowerRole(PortNum)))
#endif /* _OPTIM_CONSO*/
      {
        SignalDetachment(hcad,PortNum);
      }
      
      /* Check tCCDebounce */
      if (CAD_tCCDebounce[PortNum] > CAD_tCCDebounce_threshold) 
      {
        /* if tCCDebounce has expired state must be changed*/
        if(USBPD_PORTPOWERROLE_SRC == PE_GetPowerRole(PortNum))
        {
          hcad->cc = CCXX;
          
          switch(CAD_HW_Condition[PortNum])
          {
          case HW_Attachment:
            hcad->state = USBPD_CAD_STATE_ATTACHED;
            USBPDM1_Set_CC(PortNum, hcad->cc);
            USBPDM1_RX_EnableInterrupt(PortNum);
            break;
            
          case HW_PwrCable_NoSink_Attachment:
            hcad->state = USBPD_CAD_STATE_EMC;
            USBPDM1_Set_CC(PortNum, hcad->cc);
            USBPDM1_RX_EnableInterrupt(PortNum);
            break;
            
          case HW_PwrCable_Sink_Attachment:
            hcad->state = USBPD_CAD_STATE_ATTEMC;                  
            USBPDM1_Set_CC(PortNum, hcad->cc);
            USBPDM1_RX_EnableInterrupt(PortNum);
            break;
            
          case HW_Debug_Attachment:
            hcad->state = USBPD_CAD_STATE_DEBUG;
            break;
            
          case HW_AudioAdapter_Attachment:
            hcad->state = USBPD_CAD_STATE_ACCESSORY;
            break;
            
          case HW_Detachment:
          default:
            break;
          } /* end of switch */
          
          if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
          {
            /* Forward event information to upper layer */ 
            hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
          }
        }
        else /* Check state transition for SNK role */
        {
          if (CAD_Check_VBus(PortNum)) /* Check if Vbus is on */
          {
            hcad->cc = CCXX;
            hcad->state = USBPD_CAD_STATE_ATTACHED;
            USBPDM1_Set_CC(PortNum, hcad->cc);
            USBPDM1_RX_EnableInterrupt(PortNum);
            /* Forward event information to upper layer */
            if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
            {
              hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
            }                      
          }
        }
      }
      /* reset the flag for CAD_tPDDebounce */
      CAD_tPDDebounce_flag[PortNum] = 0; 
    }
    else /* CAD_HW_Condition[PortNum] = HW_Detachment */
    {
      /*Check tPDDebounce*/
      if (USBPD_PORTPOWERROLE_SNK == PE_GetPowerRole(PortNum))
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
          if (CAD_tPDDebounce[PortNum] > CAD_tPDDebounce_threshold) /* If elapsed */
          {
            CAD_tPDDebounce_flag[PortNum] = 0;
            SignalSwitchRole(hcad,PortNum,USBPD_CAD_STATE_SWITCH_TO_SRC);
          }
        }
      } 
      else /* (hcad->PortPowerRole != USBPD_PORTPOWERROLE_SNK)*/
      {
        SignalSwitchRole(hcad,PortNum,USBPD_CAD_STATE_SWITCH_TO_SNK);
      }
    }
    break;
    
    /*CAD STATE ATTACHED*/            
  case USBPD_CAD_STATE_ATTACHED:
    
    if(USBPD_PORTPOWERROLE_SRC == PE_GetPowerRole(PortNum))
    {
      /* Check CCx HW condition*/  
      CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));           
      
      if (CAD_HW_Condition[PortNum] == HW_Detachment ) //<----
      {
        SignalDetachment(hcad,PortNum);
        /* Forward event information to upper layer */
        if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
        {
          hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
        }              
      }
    }
    else /* hcad->PortPowerRole != USBPD_PORTPOWERROLE_SRC) */
    {
      if (CAD_Check_VBus(PortNum) == 0) /* Check if Vbus is off */
      {
        SignalDetachment(hcad,PortNum);
        /* Forward event information to upper layer */
        if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
        {
          hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
        }                 
      }
    }
    break;  
    
    /*CAD ELECTRONIC CABLE ATTACHED*/   
  case USBPD_CAD_STATE_EMC:                
    
    /* Check CCx HW condition*/  
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));           

    switch(CAD_HW_Condition[PortNum])
    {
      case HW_Detachment:
        SignalDetachment(hcad,PortNum);
        /* Forward event information to upper layer */
        if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
        {
          hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
        }              
        break;
      case HW_Attachment:
        /* Sink has been attached */
        hcad->cc = CCXX;
        hcad->OldHWCondtion = CAD_HW_Condition[PortNum];
        hcad->state = USBPD_CAD_STATE_ATTACHED_WAIT;
        /* Get the time of this event */
        CAD_tCCDebounce_start[PortNum] = HAL_GetTick()-50;
        break;
      case HW_PwrCable_Sink_Attachment:
        /* Sink has been attached */
        hcad->cc = CCXX;
        hcad->OldHWCondtion = CAD_HW_Condition[PortNum];
        hcad->state = USBPD_CAD_STATE_ATTACHED_WAIT;
        /* Get the time of this event */
        CAD_tCCDebounce_start[PortNum] = HAL_GetTick()-50;
        break;
      case HW_PwrCable_NoSink_Attachment:
      default:
        break;
      }
    break;
    
    /*CAD electronic cable with Sink ATTACHED*/                  
  case USBPD_CAD_STATE_ATTEMC:           
    
    /* Check CCx HW condition*/  
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));           
    
    if ((CAD_HW_Condition[PortNum] == HW_Detachment)
        || (CAD_HW_Condition[PortNum] == HW_PwrCable_NoSink_Attachment))
    { 
      if (CAD_HW_Condition[PortNum] == HW_PwrCable_NoSink_Attachment)
      {
        SignalSwitchRole(hcad,PortNum,USBPD_CAD_STATE_EMC);
        /* Forward event information to upper layer */
        if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
        {
          hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
        } 
      }
      else
      {
        SignalSwitchRole(hcad,PortNum,USBPD_CAD_STATE_SWITCH_TO_SNK);
        /* Forward event information to upper layer */
        if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
        {
          hcad->callback.USBPD_CAD_CallbackEvent(PortNum, USBPD_CAD_STATE_DETACHED, hcad->cc);
        }
      }
    }     
    break;
    
    /*CAD STATE AUDIO ACCESSORY ATTACHED*/            
  case USBPD_CAD_STATE_ACCESSORY:
    
    /* Check CCx HW condition*/   
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));          
    
#ifdef _ELLISYS_4_8_1
    switch(CAD_HW_Condition[PortNum])
    {
    case HW_PwrCable_NoSink_Attachment:
    case HW_Detachment:
      SignalSwitchRole(hcad,PortNum,USBPD_CAD_STATE_SWITCH_TO_SNK);
      /* Forward event information to upper layer */  
      if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
      {
        hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
      }              
    break;
    case HW_AudioAdapter_Attachment:
      break;
    default :
      break;
    }
#else
    if (CAD_HW_Condition[PortNum] != HW_AudioAdapter_Attachment) 
    {     
      SignalDetachment(hcad,PortNum);
      /* Forward event information to upper layer */  
      if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
      {
        hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
      }              
    }
#endif
    break;    
    
    /*CAD STATE DEBUG ACCESSORY MODE ATTACHED*/            
  case USBPD_CAD_STATE_DEBUG:
    
    /* Check CCx HW condition*/   
    CCXX = CAD_Check_HW(PortNum, &CAD_HW_Condition[PortNum], &CAD_SNK_Source_Current_Adv[PortNum], PE_GetPowerRole(PortNum));          
    
    if (CAD_HW_Condition[PortNum] != HW_Debug_Attachment)
    {     
      SignalDetachment(hcad,PortNum);
      /* Forward event information to upper layer */                   
      if ((hcad->callback.USBPD_CAD_CallbackEvent)!=NULL)
      {
        hcad->callback.USBPD_CAD_CallbackEvent(PortNum, hcad->state, hcad->cc);
      }              
    }
    break; 

  default :
    break;
  }
#ifdef __DEBUG_CAD
  if (CAD_HW_Condition[PortNum] != a_HWCondition[indexHWcond2-1])
  {
    a_HWCondition[indexHWcond2++] = CAD_HW_Condition[PortNum];
    USBPD_TRACE_Add(USBPD_TRACE_CAD_LOW, PortNum, hcad->state, NULL, 0);
  }
#endif  
  
}

static inline void SignalDetachment(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum)
{
  HW_SignalDetachment(PortNum,hcad->cc);
  hcad->cc = CCNONE;
  hcad->state = USBPD_CAD_STATE_DETACHED;
}

static inline void SignalSwitchRole(USBPD_CAD_HandleTypeDef* hcad, uint8_t PortNum, USBPD_CAD_STATE State)
{
  HW_SignalDetachment(PortNum,hcad->cc);
  hcad->cc = CCNONE;
  hcad->state = State;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
