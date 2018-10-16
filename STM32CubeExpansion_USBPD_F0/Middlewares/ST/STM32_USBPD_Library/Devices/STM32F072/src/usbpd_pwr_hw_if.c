/**
  ******************************************************************************
  * @file    usbpd_pwr_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface functions.
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
#include "p-nucleo-usb001.h"
#include "usbpd_hw_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PWR_HW_IF
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Variable containing ADC conversions results */
extern uint32_t             ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];
/* Array of power selection pins used by @ref P-NUCLEO-USB001 */
extern USBPD_BSP_GPIOPins_TypeDef USBPDM1_POWSELs[USBPDM1_POWSELn];
/* Array of GPIO used by @ref P-NUCLEO-USB001 */
extern USBPD_BSP_GPIOPins_TypeDef USBPDM1_GPIOs[USBPDM1_GPIOn];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
HAL_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage)
{
  /* Section below has been commented for the P-Nucleo Demo */
  /*
  uint32_t off = (PortNum == 0) ? 0 : 2;
  switch (voltage)
  {
  case 12000:
    HAL_GPIO_WritePin(USBPDM1_POWSELs[0+off].GPIOx, USBPDM1_POWSELs[0+off].GPIO_Pin, GPIO_PIN_SET);
    break;
  case 5000:
  default:
    HAL_GPIO_WritePin(USBPDM1_POWSELs[0+off].GPIOx, USBPDM1_POWSELs[0+off].GPIO_Pin, GPIO_PIN_RESET);
    break;
  }
  */
  return HAL_OK;
}

uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum)
{
  return (uint16_t)MVOLT(ADCxConvertedValues[VBUS_INDEX(PortNum)]);
}

int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum)
{
  int16_t signed_current;
  signed_current = (int16_t)(MAMP(ADCxConvertedValues[IBUS_INDEX(PortNum)]));
  if (signed_current < 0)
    return (- signed_current);
  else
    return signed_current;
}

HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState state, CCxPin_TypeDef Cc, uint32_t VconnState, USBPD_PortPowerRole_TypeDef role)
{
#if defined(_VCONN_SUPPORT)  
  /* Turn OFF VConn */
  if((1 == VconnState) && (state == USBPD_DISABLE))
  {
      HW_IF_Disable_VConn(PortNum, Cc);
  }
#endif
  
  /* Turns on or off the VBus */
#if (USBPD_PORT_COUNT == 2)
  if (PortNum == 0)
  {
#endif
    (state == USBPD_ENABLE) ? USBPDM1_GPIO_On(PWREN_P0) : USBPDM1_GPIO_Off(PWREN_P0);
    if (role == USBPD_PORTPOWERROLE_SRC)
    {
      if (state == USBPD_ENABLE)
      {
        USBPDM1_GPIO_Off(PWRDIS_P0);
      }
      else
      {
        USBPDM1_GPIO_On(PWRDIS_P0);
      }
    }
    else
    {
      /* To be sure the discharge is off */
      USBPDM1_GPIO_Off(PWRDIS_P0);
    }
#if (USBPD_PORT_COUNT == 2)
  }
  else
  {
    if (state == USBPD_ENABLE)
    {
      USBPDM1_GPIO_On(PWREN_P1);
    }
    else
    {
      USBPDM1_GPIO_Off(PWREN_P1);
    }
    if (role == USBPD_PORTPOWERROLE_SRC)
    {
      if (state == USBPD_ENABLE)
      {
        USBPDM1_GPIO_Off(PWRDIS_P1);
      }
      else
      {
        USBPDM1_GPIO_On(PWRDIS_P1);
      }
    }
    else
    {
      /* To be sure the discharge is off */
      USBPDM1_GPIO_Off(PWRDIS_P1);
    }
  }
#endif

#if defined(_VCONN_SUPPORT)  
  if((1 == VconnState) && (state == USBPD_ENABLE))
  {
      /* Vconn on must de done max 2ms after VBUS on */
      HW_IF_Enable_VConn(PortNum, Cc);
  }
#endif
  
  return HAL_OK;
}

void HW_IF_PWR_DisableDischPath(uint8_t PortNum)
{
  USBPDM1_GPIO_Off((USBPD_PORT_0 == (PortNum))? PWRDIS_P0 : PWRDIS_P1);
}

USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum)
{
  /* Checks if the VBus is enabled */
  USBPD_FunctionalState ret = USBPD_DISABLE;
  if (PortNum == 0)
  {
    ret = (HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P0].GPIOx, USBPDM1_GPIOs[PWREN_P0].GPIO_Pin) == GPIO_PIN_SET) ? USBPD_ENABLE : USBPD_DISABLE ;
  }

#if (USBPD_PORT_COUNT >= 2)
  if (PortNum == 1)
  {
    ret = (HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P1].GPIOx, USBPDM1_GPIOs[PWREN_P1].GPIO_Pin) == GPIO_PIN_SET) ? USBPD_ENABLE : USBPD_DISABLE ;
  }
#endif
  return ret;
}

/**
  * @brief  Enables the VConn on the port.
  * @param  PortNum Port number
  * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
  * @retval None
  */
void HW_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef CC)
{
  switch (CC)
  {
      /* Turns on the VConn on the selected CC */
    case CC1:
      USBPDM1_GPIO_Off(ENCC1_PIN(PortNum));
      break;
    case CC2:
      USBPDM1_GPIO_Off(ENCC2_PIN(PortNum));
      break;
    case CCNONE:
      /* Turns off the VConn on all CC */
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC1_PIN(PortNum));
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));

      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC2_PIN(PortNum));
      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
      break;
  }
}

/**
  * @brief  Disable the VConn on the port.
  * @param  PortNum Port number
  * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
  * @retval None
  */
void HW_IF_Disable_VConn(uint8_t PortNum, CCxPin_TypeDef CC)
{
  switch (CC)
  {
      /* Turns off the VConn on the selected CC */
    case CC1:
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC1_PIN(PortNum)); /* Mandatory to pass tests */
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));  /* Mandatory to pass tests */
      break;
    case CC2:
      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC2_PIN(PortNum)); /* Mandatory to pass tests */
      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));  /* Mandatory to pass tests */
      break;
    case CCNONE:
      /* Turns off the VConn on all CC */
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC1_PIN(PortNum));
      USBPDM1_GPIO_On(ENCC1_PIN(PortNum));

      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
      USBPDM1_GPIO_Off(ENCC2_PIN(PortNum));
      USBPDM1_GPIO_On(ENCC2_PIN(PortNum));
      break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

