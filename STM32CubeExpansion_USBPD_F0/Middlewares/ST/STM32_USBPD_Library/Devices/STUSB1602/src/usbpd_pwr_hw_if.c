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
#include "usbpd_hw_if.h"
#include "p-nucleo-usb002.h"
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"

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
#define USBPD_POWSELn    ((USBPD_PORT_COUNT) * 2)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t ADCxConvertedValues[];

/**
  * @brief Array of power selection pins used by P-NUCLEO-USB002
  */
static const USBPD_BSP_GPIOPins_TypeDef USBPD_POWSELs[USBPD_POWSELn] =
{
  USBPD_BSP_PIN(GPIOB,7),
  USBPD_BSP_PIN(GPIOB,6),
#if USBPD_PORT_COUNT == 2
  USBPD_BSP_PIN(GPIOC,1),
  USBPD_BSP_PIN(GPIOC,9),
#endif
};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Inititialization of the Power Pins.
  * @retval HAL Status
  */
HAL_StatusTypeDef HW_IF_PWR_DigitalGPIO_Init()
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t index = 0;
  for(index=0;index<USBPD_POWSELn;index++)
  {
    USBPD_BSP_GPIOPins_TypeDef gpio = USBPD_POWSELs[index];

    /* Configure the powsels pin */
    GPIO_InitStruct.Pin = gpio.GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);

    /* Turn the pin off */
    USBPD_HW_IF_GPIO_Off(gpio);
  }

  return HAL_OK;
}


/**
  * @brief  Set the VBUS voltage level on a specific port.
  * @param  PortNum  port index
  * @param  voltage  voltage value.
  * @retval HAL status
  */
USBPD_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage)
{
 USBPD_StatusTypeDef ret = USBPD_OK;
#if _PPS==USBPD_FALSE
  /* configuration for STCH2 Board, enabled only profile 5V and 9V
   * Connections
   * +----------------+-------------------------------------------+
   * |   J4 STCH2     |     C4 MB1303/MB1257                      |
   * +----+-----------+----+--------------------------------------+
   * |PIN#| PIN Name  |PIN#|PIN Name                              |
   * +----+-----------+----+--------------------------------------+
   * | 1  | Vout      |1-3 |POWCNN1 (VBus)                        |
   * | 2  | SEL1      |9   |POWCNN9 / CN7.21 (PB7)    (open drain)|
   * | 3  | SEL2      |11  |POWCNN11 / CN10.17 (PB6)  (open drain)|
   * | 4  | GND       |5-7 |GND                                   |
   * +----+-----------+----+--------------------------------------+
   *
   * STCH2 Voltage Output
   * +------+------+------+
   * | SEL2 | SEL1 | VOUT |
   * +------+------+------+
   * |  0   |  0   |  5V  |
   * |  0   |  1   |  9V  |
   * |  1   |  -   | 12V  |
   * +------+------+------+
   */

  uint32_t offset_port = (PortNum == 0) ? 0 : 2;


#ifdef CONF_NORMAL
  /* force low both sel pins */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

#ifdef CONF_DEMO
  /* force low both sel pins */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

#ifdef CONF_DEMO_FPGA
  if (voltage != 5000 && voltage != 9000)
  {
    return USBPD_ERROR;
  }

  ret = (USBPD_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, voltage, 10, 10);

  /* force SEL2 resetted (no 12V) */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);

  /* set SEL1 according to the selected voltage and the table STCH2 Voltage Output */
  USBPD_HW_IF_GPIO_Set(USBPD_POWSELs[offset_port+1], voltage == 9000 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

#else /* case of PPS with VVAR*/
  STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);

  ret=(USBPD_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum),voltage);

#endif
  return ret;
}

/**
  * @brief  Get the voltage level on a specified port.
  * @param  PortNum port index
  * @retval The voltage value
  */
uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum)
{
#ifdef __VVAR
   return (uint16_t)MVOLT(ADCxConvertedValues[VBUS_INDEX(PortNum)]);
#else
  return (uint16_t)(STUSB1602_VBUS_Select_Status_Get(STUSB1602_I2C_Add(PortNum)));
#endif
}

/**
  * @brief  Get the voltage level on a specified port (only from 1602 reg).
  * @param  PortNum port index
  * @retval The voltage value
  */
uint16_t HW_IF_PWR_GetVoltage_from_reg(uint8_t PortNum)
{
  return (uint16_t)(STUSB1602_VBUS_Select_Status_Get(STUSB1602_I2C_Add(PortNum)));

}


/**
  * @brief  Get the current value on a specified port.
  * @param  PortNum port index
  * @retval The voltage value
  */
int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum)
{
  int16_t signed_current;
  signed_current = (int16_t)(MAMP(ADCxConvertedValues[IBUS_INDEX(PortNum)]) +23);
  if (signed_current < 0)
    return (- signed_current);
  else
    return signed_current;

}


/**
  * @brief  Enable the VBUS on a specified port.
  * @param  PortNum    The handle of the port.
  * @param  state      USBPD state
  * @param  Cc         CC line based on @ref CCxPin_TypeDef
  * @param  VconnState VCONN state
  * @param  role       Power role
  * @retval USBPD status
  */
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState state, CCxPin_TypeDef Cc, uint32_t VconnState, USBPD_PortPowerRole_TypeDef role)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
#if defined(CONF_NORMAL) || defined(CONF_DEMO_FPGA)
  uint32_t offset_port = (PortNum == 0) ? 0 : 2;
#endif

  if (state == USBPD_DISABLE)
  {
    ret = HAL_OK; /* To allow compatibility with other type-c controllers */
  }
  else
  {
    ret = HAL_OK; /* To allow compatibility with other type-c controllers */
  }

#ifdef CONF_NORMAL
  /* both pin are set off */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

#ifdef CONF_DEMO_FPGA
  ret = STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, 5000, 10, 10);

  /* force SEL2 resetted (no 12V) */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);

  /* set SEL1 according to the selected voltage and the table STCH2 Voltage Output */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

  return ret;
}

USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum)
{
  /* Checks if the VBus is enabled */
  USBPD_FunctionalState ret = USBPD_DISABLE;
//  if (PortNum == 0)
//  {
//    ret = (HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P0].GPIOx, USBPDM1_GPIOs[PWREN_P0].GPIO_Pin) == GPIO_PIN_SET) ? USBPD_ENABLE : USBPD_DISABLE ;
//  }
//
//#if (USBPD_PORT_COUNT >= 2)
//  if (PortNum == 1)
//  {
//    ret = (HAL_GPIO_ReadPin(USBPDM1_GPIOs[PWREN_P1].GPIOx, USBPDM1_GPIOs[PWREN_P1].GPIO_Pin) == GPIO_PIN_SET) ? USBPD_ENABLE : USBPD_DISABLE ;
//  }
//#endif
  return ret;
}

/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  PortNum    The handle of the port.
  * @retval FunctionalState
  */
FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum)
{
  /* Checks if the VBus is enabled */
  FunctionalState Status_ret = (FunctionalState) STUSB1602_VBUS_SRC_State_Get(STUSB1602_I2C_Add(PortNum));

  return Status_ret;
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

