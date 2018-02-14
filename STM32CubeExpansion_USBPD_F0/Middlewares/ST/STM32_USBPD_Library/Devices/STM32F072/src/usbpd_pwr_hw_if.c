/**
  ******************************************************************************
  * @file    usbpd_pwr_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface functions.
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
  return (uint16_t)MVOLT( ADCxConvertedValues[VBUS_INDEX(PortNum)] );
}

int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum)
{
  return (int16_t)MAMP( ADCxConvertedValues[IBUS_INDEX(PortNum)] );
}

HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, FunctionalState state, USBPD_PortPowerRole_TypeDef role)
{
  /* Turns on or off the VBus */
#if (USBPD_PORT_COUNT == 2)
  if (PortNum == 0)
  {
#endif
#if (USBPD_USED_PORT == 1)
    (state == ENABLE) ? USBPDM1_GPIO_On(PWREN_P1) : USBPDM1_GPIO_Off(PWREN_P1);
    if ( (role == USBPD_PORTPOWERROLE_SRC) || (role == USBPD_PORTPOWERROLE_DRP_SRC) )
    {
      (state == ENABLE) ? USBPDM1_GPIO_Off(PWRDIS_P1) : USBPDM1_GPIO_On(PWRDIS_P1);
    }
    else
    {
      /* To be sure the discharge is off */
      USBPDM1_GPIO_Off(PWRDIS_P1);
    }
#else
    (state == ENABLE) ? USBPDM1_GPIO_On(PWREN_P0) : USBPDM1_GPIO_Off(PWREN_P0);
    if ( (role == USBPD_PORTPOWERROLE_SRC) || (role == USBPD_PORTPOWERROLE_DRP_SRC) )
    {
      (state == ENABLE) ? USBPDM1_GPIO_Off(PWRDIS_P0) : USBPDM1_GPIO_On(PWRDIS_P0);
    }
    else
    {
      /* To be sure the discharge is off */
      USBPDM1_GPIO_Off(PWRDIS_P0);
    }
#endif
#if (USBPD_PORT_COUNT == 2)
  }
  else
  {
    if (state == ENABLE)
    {
      USBPDM1_GPIO_On(PWREN_P1);
    }
    else
    {
      USBPDM1_GPIO_Off(PWREN_P1);
    }
    if ( (role == USBPD_PORTPOWERROLE_SRC) || (role == USBPD_PORTPOWERROLE_DRP_SRC) )
    {
      (state == ENABLE) ? USBPDM1_GPIO_Off(PWRDIS_P1) : USBPDM1_GPIO_On(PWRDIS_P1);
    }
    else
    {
      /* To be sure the discharge is off */
      USBPDM1_GPIO_Off(PWRDIS_P1);
    }
  }
#endif
  return HAL_OK;
}

FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum)
{
  /* Checks if the VBus is enabled */
  FunctionalState ret = DISABLE;
#if (USBPD_PORT_COUNT == 2)
  if (PortNum == 0)
  {
#endif
#if ((USBPD_PORT_COUNT == 1) && (USBPD_USED_PORT == 1))
    ret = ( HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P1].GPIOx,USBPDM1_GPIOs[PWREN_P1].GPIO_Pin) == GPIO_PIN_SET ) ? ENABLE : DISABLE ;
#else
    ret = ( HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P0].GPIOx,USBPDM1_GPIOs[PWREN_P0].GPIO_Pin) == GPIO_PIN_SET ) ? ENABLE : DISABLE ;
#endif
#if (USBPD_PORT_COUNT == 2)
  }
  else if (PortNum == 1)
  {
    ret = ( HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P1].GPIOx,USBPDM1_GPIOs[PWREN_P1].GPIO_Pin) == GPIO_PIN_SET ) ? ENABLE : DISABLE ;
  }
#endif
  return ret;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
