/**
  ******************************************************************************
  * @file    usbpd_tcpci.h
  * @author  MCD Application Team
  * @brief   Header file of Policy Engine module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBPD_TCPCI_H
#define __USBPD_TCPCI_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup STM32F0XX_NUCLEO
  * @{
  */

/** @addtogroup STM32F0XX_NUCLEO_TCPCI
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"
#include "stm32f0xx.h"

/* Exported define -----------------------------------------------------------*/ 
/** @defgroup STM32F0XX_NUCLEO_TCPI TCPI TCPCI exported Constants
  * @{
  */
/*****************************************************************************
   I2C definition
 ******************************************************************************/
#define I2Cx             I2C2
/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 48 MHz */
/* This example use TIMING to 0x00A51314 to reach 1 MHz speed (Rise time = 100 ns, Fall time = 100 ns) */
//#define I2C_TIMING        0x00901850
#define I2C_TIMING        0x00200B19

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_10
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF1_I2C2

/*****************************************************************************
   RCC definition
 ******************************************************************************/
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

/*****************************************************************************
   Alert PIN
 ******************************************************************************/
#if defined(_TCPM_FUSB302)
#define ALERT_PORT0_GPIO_CLK_ENABLE   __HAL_RCC_GPIOB_CLK_ENABLE
#define ALERT_PORT0_GPIO_PORT         GPIOB
#define ALERT_PORT0_GPIO_PIN          GPIO_PIN_1
#define ALERT_PORT0_EXTI_IRQn         EXTI0_1_IRQn
#define ALERT_PORT0_EXTI_IRQHandler   EXTI0_1_IRQHandler
#else
#define ALERT_PORT0_GPIO_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE
#define ALERT_PORT0_GPIO_PORT         GPIOA
#define ALERT_PORT0_GPIO_PIN          GPIO_PIN_8
#define ALERT_PORT0_EXTI_IRQn         EXTI4_15_IRQn
#define ALERT_PORT0_EXTI_IRQHandler   EXTI4_15_IRQHandler
#endif /* _TCPM_FUSB302 */

#ifdef  TCPC_FL7101
#define ALERT_PORT1_GPIO_CLK_ENABLE   __HAL_RCC_GPIOB_CLK_ENABLE
#define ALERT_PORT1_GPIO_PORT         GPIOB
#else
#define ALERT_PORT1_GPIO_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE
#define ALERT_PORT1_GPIO_PORT         GPIOA
#endif /* TCPC_FL7101 */
#define ALERT_PORT1_GPIO_PIN          GPIO_PIN_2
#define ALERT_PORT1_EXTI_IRQn         EXTI2_3_IRQn
#define ALERT_PORT1_EXTI_IRQHandler   EXTI2_3_IRQHandler

#define ALERT_GPIO_IRQHANDLER(_PORT_) (((_PORT_) == USBPD_PORT_0) ? ALERT_PORT0_EXTI_IRQn : ALERT_PORT1_EXTI_IRQn)

/*****************************************************************************
  VBUS PIN
******************************************************************************/
#define VBUS_GPIO_PORT0_CLK_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE
#define VBUS_GPIO_PORT0                 GPIOC
#define VBUS_GPIO_PIN_PORT0             GPIO_PIN_3
#if USBPD_PORT_COUNT==2
#define VBUS_GPIO_PORT1_CLK_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE/* TBC*/
#define VBUS_GPIO_PORT1                 GPIOC /* TBC*/
#define VBUS_GPIO_PIN_PORT1             GPIO_PIN_4/* TBC*/
#endif
/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32F0XX_NUCLEO_TCPI_Functions
  * @{
  */

/** @addtogroup STM32F0XX_NUCLEO_TCPI_Functions_Grp1
  * @{
  */

/**
  * @brief  Initialize ALERT pin
  * @retval None
  */
void                USBPD_TCPI_AlertInit(void);

/**
  * @brief  Initialize TCPCI link
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_Init(void);

/**
  * @brief  Un-Initialize TCPCI 
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_DeInit(void);

/**
  * @brief  Get the TCPC Drivers
  * @param  PortNum     Index of current used port
  * @param  TCPC_Driver Pointer on the TCPC drivers
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_GetDevicesDrivers(uint8_t PortNum, TCPC_DrvTypeDef **TCPC_Driver);

#if defined(_TCPM_FUSB302)
/**
  * @brief  Search write TCPC address
  * @param  portnum Index of current used port
  * @param  RegisterId register Id
  * @param  ptrData data pointer
  * @param  datasize    Size of the data
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_SearchAddress(uint8_t Port, uint8_t RegisterId, uint8_t *prtData, uint8_t datasize);
#endif /* _TCPM_FUSB302 */
/**
  * @brief  TCPCI read register
  * @param  portnum Index of current used port
  * @param  RegisterId  register Id
  * @param  datasize    Size of the data
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_ReadRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize);

/**
  * @brief  TCPCI write register
  * @param  port        Index of current used port
  * @param  RegisterId  register Id
  * @param  ptrData     data pointer
  * @param  datasize    Size of the data
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_WriteRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize);

/**
  * @brief  TCPCI sends Transmitter buffer message
  * @param  Port              Index of current used port
  * @param  RegisterId        register Id
  * @param  TransmitByteCount Number of bytes to transmit
  * @param  Header            PD header to transmit
  * @param  pData             data pointer
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_SendTransmitBuffer(uint32_t Port, uint8_t RegisterId, uint8_t TransmitByteCount, uint16_t Header, uint8_t *pData);

/**
  * @brief  TCPCI receives buffer message
  * @param  Port        Index of current used port
  * @param  RegisterId  register Id
  * @param  Buffer      Pointer of received data
  * @param  SOPType     Pointer of received SOP type
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_ReceiveBuffer(uint32_t Port, uint8_t registerId, uint8_t *Buffer, uint8_t *SOPType);

/**
  * @brief  TCPCI delay
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void                USBPD_TCPCI_Delay(uint32_t Delay);

/**
  * @}
  */

/** @addtogroup STM32F0XX_NUCLEO_TCPI_Functions_Grp2
  * @{
  */

/**
  * @brief  Enable the VBUS on a specified port.
  * @param  PortNum The port handle.
  * @param  state   ENABLE or DISABLE.
  * @param  role    The role of the port.
  * @retval HAL status
  */
USBPD_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState state, CCxPin_TypeDef Cc, uint32_t VconnState, USBPD_PortPowerRole_TypeDef role);

/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  PortNum The port handle.
  * @retval FunctionalState
  */
USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum);

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  PortNum The port handle.
  * @param  voltage voltage value to be set.
  * @retval HAL status
  */
USBPD_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage);

/**
  * @brief  Get the voltage level on a specified port.
  * @param  PortNum The port handle.
  * @retval The voltage value
  */
uint16_t            HW_IF_PWR_GetVoltage(uint8_t PortNum);

/**
  * @brief  Get the current level on a specified port.
  * @param  PortNum The port handle.
  * @retval The current value
  */
int16_t             HW_IF_PWR_GetCurrent(uint8_t PortNum);

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

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_TCPCI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
