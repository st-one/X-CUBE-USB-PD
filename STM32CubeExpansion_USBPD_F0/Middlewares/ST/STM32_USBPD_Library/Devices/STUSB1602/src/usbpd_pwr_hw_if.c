/**
  ******************************************************************************
  * @file    usbpd_pwr_hw_if.c
  * @author  System Lab
  * @brief   This file contains power hardware interface functions.
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
#include "STUSB1602_Driver.h"
#include "STUSB1602_Driver_Conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USBPD_POWSELn    ((USBPD_PORT_COUNT) * 2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Array of power selection pins used by @ref P-NUCLEO-USB001 */
static const USBPD_BSP_GPIOPins_TypeDef USBPD_POWSELs[USBPD_POWSELn] =
{
  USBPD_BSP_PIN(GPIOB,7),
  USBPD_BSP_PIN(GPIOB,6),
#if USBPD_PORT_COUNT == 2
  USBPD_BSP_PIN(GPIOC,1),
  USBPD_BSP_PIN(GPIOC,9),
#endif
};

/* Note (GN):
extern USBPDM1_GPIOPins_TypeDef USBPDM1_GPIOs[USBPDM1_GPIOn];
*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Inititialization of the Power Pins.
  */
HAL_StatusTypeDef HW_IF_PWR_DigitalGPIO_Init()
{
  USBPD_BSP_GPIOPins_TypeDef gpio;
  GPIO_InitTypeDef  GPIO_InitStruct;
#ifndef CONF_POWERBANK_POWERSIMPLE
  uint8_t index = 0;
  for(index=0;index<USBPD_POWSELn;index++)
  {
    gpio = USBPD_POWSELs[index];
    
    /* Configure the powsels pin */
    GPIO_InitStruct.Pin = gpio.GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);
    
    /* Turn the pin off */
    USBPD_HW_IF_GPIO_Off(gpio);
  }
#else  
  
  /* PB7 SEL1 */
  gpio = USBPD_POWSELs[0];
  
  /* Configure the powsels pin */
  GPIO_InitStruct.Pin = gpio.GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);

  /* Turn the pin off */
  USBPD_HW_IF_GPIO_Off(gpio);
  

  /* PB6 EN1 open drain */
  gpio = USBPD_POWSELs[1];
  
  /* Configure the powsels pin */
  GPIO_InitStruct.Pin = gpio.GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);

#if USBPD_PORT_COUNT == 2
#warning "missing initialization pin port 1"
#endif  

#endif

  /* Turn the pin off */
  USBPD_HW_IF_GPIO_Off(gpio);
  
  return HAL_OK;
}

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  PortNum:    port index
  * @param  voltage:  voltage value.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage)
{

  HAL_StatusTypeDef ret = HAL_OK;
  uint32_t offset_port = (PortNum == 0) ? 0 : 2;

#if defined(CONF_NORMAL) || defined(CONF_DEMO)
  /* force low both sel pins */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

#ifdef CONF_STCH02
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
  if (voltage != 5000 && voltage != 9000)
  {
    return HAL_ERROR;
  }

  ret = (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, voltage, 10, 10);

  /* force SEL2 resetted (no 12V) */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  
  /* set SEL1 according to the selected voltage and the table STCH2 Voltage Output */
  USBPD_HW_IF_GPIO_Set(USBPD_POWSELs[offset_port+1], voltage == 9000 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

#ifdef CONF_POWERBANK_POWERSIMPLE
#if !defined(CONF_POWERBANK_POWERSIMPLE_VOLTAGE) || (CONF_POWERBANK_POWERSIMPLE_VOLTAGE < 5000 || CONF_POWERBANK_POWERSIMPLE_VOLTAGE > 20000)
  #error "Please define CONF_POWERBANK_POWERSIMPLE_VOLTAGE in usbpd_conf.h as valid value"
#endif

  /* missing a check of the voltage */
  if (voltage != 5000 && voltage != CONF_POWERBANK_POWERSIMPLE_VOLTAGE)
  {
    return HAL_ERROR;
  }

  ret = (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, voltage, 10, 10);

  /* set SEL1 according to the selected voltage and the table STCH2 Voltage Output */
  USBPD_HW_IF_GPIO_Set(USBPD_POWSELs[offset_port], voltage == CONF_POWERBANK_POWERSIMPLE_VOLTAGE ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* force SEL2 <=> EN1 low */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif
  
  return ret;
}

/**
  * @brief  Get the voltage level on a specified port.
  * @param  PortNum: port index
  * @retval The voltage value 
  */
uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum)
{
  /* To be implemented */
  return 0;
}


/**
  * @brief  Get the current value on a specified port.
  * @param  PortNum: port index
  * @retval The voltage value 
  */
uint16_t HW_IF_PWR_GetCurrent(uint8_t PortNum)
{
  /* To be implemented */
  return 0;
}


/**
  * @brief  Enable the VBUS on a specified port.
  * @param  PortNum:    The handle of the port.
  * @param  state:    ENABLE or DISABLE.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, FunctionalState state, USBPD_PortPowerRole_TypeDef role)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
#if defined(CONF_NORMAL) || defined(CONF_STCH02) || defined(CONF_POWERBANK_POWERSIMPLE)
  uint32_t offset_port = (PortNum == 0) ? 0 : 2;
#endif /*CONF_NORMAL || CONF_STCH02 || CONF_POWERBANK_POWERSIMPLE */

  if (state == DISABLE)
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
  
#ifdef CONF_STCH02
  ret = STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, 5000, 10, 10);
  
  /* force SEL2 resetted (no 12V) */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  
  /* set SEL1 according to the selected voltage and the table STCH2 Voltage Output */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif
  
#ifdef CONF_POWERBANK_POWERSIMPLE
  ret = STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, 5000, 10, 10);

  /* SEL1 set 12V and reset to 5V */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);

  /* EN1 is EN1 always enable, we can change according to the on/off */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif
  
return ret;
}


/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  PortNum:    The handle of the port.
  * @retval FunctionalState
  */
FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum)
{
  /* Checks if the VBus is enabled */
  FunctionalState Status_ret = (FunctionalState) STUSB1602_VBUS_SRC_State_Get(STUSB1602_I2C_Add(PortNum));
  
  return Status_ret;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
