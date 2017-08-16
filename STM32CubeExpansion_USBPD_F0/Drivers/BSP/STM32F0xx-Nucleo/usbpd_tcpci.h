/**
  ******************************************************************************
  * @file    usbpd_tcpci.h
  * @author  MCD Application Team
  * @version V0.0.3
  * @date    12-April-2016
  * @brief   Header file of Policy Engine module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBPD_TCPCI_H
#define __USBPD_TCPCI_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"  
#if !defined(USBPD_TCPM_MODULE_ENABLED)
#include "usbpd_conf.h"
#endif /* USBPD_TCPM_MODULE_ENABLED */

/* Exported define -----------------------------------------------------------*/ 
/** @defgroup STM32F0XX_NUCLEO_TCPI TCPI Constants
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
#ifdef USBPD_TCPC
#define I2C_ADDRESS       0x0C
#else
#define I2C_ADDRESS       0xA0
#endif

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_10
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF1_I2C2

/*****************************************************************************
   RCC definition
 ******************************************************************************/
#define I2C_CLOCKSOURCE
#ifdef I2C_CLOCKSOURCE
#define RCC_I2CxCLKSOURCE_SYSCLK   RCC_I2C2CLKSOURCE_SYSCLK
#define RCC_PERIPHCLK_I2Cx         RCC_PERIPHCLK_I2C1
#endif

#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx's NVIC */
#define I2Cx_IRQn                    I2C2_IRQn
#define I2Cx_IRQHandler              I2C2_IRQHandler

/*****************************************************************************
   Alert PIN
 ******************************************************************************/
#ifdef USBPD_TCPC
#define ALERT_GPIO_CLK_ENABLE   __HAL_RCC_GPIOC_CLK_ENABLE
#define ALERT_GPIO_PORT         GPIOC
#define ALERT_GPIO_PIN          GPIO_PIN_6
#define ALERT_EXTI_IRQn         EXTI4_15_IRQn
#define ALERT_EXTI_IRQHandler   EXTI4_15_IRQHandler
#else
#define ALERT_GPIO_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE
#define ALERT_GPIO_PORT         GPIOA
#define ALERT_GPIO_PIN          GPIO_PIN_8
#define ALERT_EXTI_IRQn         EXTI4_15_IRQn
#define ALERT_EXTI_IRQHandler   EXTI4_15_IRQHandler
#endif

#define ALERT_PORT1_GPIO_CLK_ENABLE   __HAL_RCC_GPIOA_CLK_ENABLE
#define ALERT_PORT1_GPIO_PORT         GPIOA
#define ALERT_PORT1_GPIO_PIN          GPIO_PIN_2
#define ALERT_PORT1_EXTI_IRQn         EXTI2_3_IRQn
#define ALERT_PORT1_EXTI_IRQHandler   EXTI2_3_IRQHandler

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
    
#ifdef USBPD_TCPC 
typedef enum {
  TCPC_READ  = 0x01,
  TCPC_WRITE = 0x02
} USBPD_TCPC_Direction;
#endif

/* Exported typedef ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup TCPCI_Exported_Functions
  * @{
  */
USBPD_StatusTypeDef USBPD_TCPCI_Init(void);
USBPD_StatusTypeDef USBPD_TCPCI_DeInit(void);

#ifdef USBPD_TCPC
USBPD_StatusTypeDef USBPD_TCPCI_ListenPort(uint8_t port, uint8_t *direction, uint8_t *registerid, uint8_t **ptr);
#else
USBPD_StatusTypeDef USBPD_TCPCI_RegisterPort(uint8_t port, uint32_t i2c_addr);
USBPD_StatusTypeDef USBPD_TCPCI_ReadRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize);
USBPD_StatusTypeDef USBPD_TCPCI_WriteRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize);
USBPD_StatusTypeDef USBPD_TCPCI_SendTransmitBuffer(uint32_t Port, uint8_t RegisterId, uint8_t TransmitByteCount, uint16_t Header, uint8_t *pData);
USBPD_StatusTypeDef USBPD_TCPCI_ReceiveBuffer(uint32_t Port, uint8_t registerId, uint8_t *Buffer, uint8_t *SOPType);
USBPD_StatusTypeDef USBPD_TCPCI_WriteBuffer(uint8_t Port, uint8_t *prtData, uint16_t DataSize);
void USBPD_TCPCI_Delay(uint32_t Delay);
#endif

/** @defgroup STM32F0XX_NUCLEO_Exported_Functions Exported Functions
  * @{
  */
void BSP_TCPI_AlertInit(void);
#ifdef USBPD_TCPC
void BSP_TCPI_AlertON(void);
void BSP_TCPI_AlertOFF(void);
#endif
void BSP_TCPI_VBUS_Init(void);
void BSP_TCPI_VBUS_ON(uint32_t Port);
void BSP_TCPI_VBUS_OFF(uint32_t Port);
void BSP_USBPD_SetVoltage(uint32_t Port, uint32_t Voltage);
uint16_t BSP_USBPD_GetVoltage(uint32_t Port);
uint16_t BSP_USBPD_GetCurrent(uint32_t Port);
#ifdef HAL_I2C_MODULE_ENABLED
uint32_t BSP_TCPI_Init(I2C_HandleTypeDef *hi2c);
uint32_t BSP_TCPI_DeInit(I2C_HandleTypeDef *hi2c);
#endif
/**
  * @}
  */ 

/**
  * @}
  */

#endif /* __USBPD_TCPCI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
