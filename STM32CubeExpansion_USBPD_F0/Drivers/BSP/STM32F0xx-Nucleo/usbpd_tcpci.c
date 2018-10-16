/**
  ******************************************************************************
  * @file    usbpd_tcpci.c
  * @author  MCD Application Team
  * @brief   This file provides the low layer firmware functions to manage the 
  *          following functionalities of the tcpc interface:
  *           - Initialization and Configuration of I2C
  *           - USB PD port registration
  *           - TCPC register read/write
  *           - TCPC message send/get
  *           - Callback registration 
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                      TCPC interface
  *          =================================================================== 
  *           This driver manages the:
  *           - Initialization and Configuration of Policy Engine layer
  *           - USB PD Command transfer (USB PD requests management)
  *           - OUT/IN data transfer
  *           - Error management  
  *
  *  @endverbatim
  *
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

/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup STM32F0XX_NUCLEO
  * @{
  */

/** @addtogroup STM32F0XX_NUCLEO_TCPCI
  * @{
  */

#include "usbpd_core.h"
#ifdef  TCPC_FL7101
#include "fl7101.h"
#include "pv2105.h"
#else
#if defined(_TCPM_VP230)
#include "vp230.h"
#elif defined(_TCPM_FUSB302)
#include "fusb302.h"
#else
#include "fusb305.h"
#endif /* _TCPM_VP230 */
#endif  /* TCPC_FL7101 */
#include "usbpd_tcpci.h"
#include "cmsis_os.h"
#include "string.h" 
#include "stm32f0xx_hal_i2c.h"
#if defined(_TRACE)
#include "usbpd_trace.h"
#endif /* _TRACE */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t i2c_addr;
} t_port_mapping;

/* Private define ------------------------------------------------------------*/
/** @defgroup STM32F0XX_NUCLEO_TCPI_Private_define TCPI Private define
  * @{
  */
#ifdef TCPC_FL7101
#define I2C_SLAVE_ADDRESS_PORT0    0x70u
#if USBPD_PORT_COUNT == 2
#define I2C_SLAVE_ADDRESS_PORT1       0x70u
#endif /* USBPD_PORT_COUNT == 2 */
#else
#if defined(_TCPM_VP230)
#define I2C_SLAVE_ADDRESS_PORT0       0x40u
#elif defined(_TCPM_FUSB302)
#define I2C_SLAVE_ADDRESS_PORT0       FUSB302_I2C_SLAVE_ADDR
#else
#define I2C_SLAVE_ADDRESS_PORT0       0xA0u
#endif /* _TCPM_VP230 */
#if USBPD_PORT_COUNT == 2
#define I2C_SLAVE_ADDRESS_PORT1       0xA0u
#endif /* USBPD_PORT_COUNT == 2 */
#endif /* TCPC_FL7101 */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup STM32F0XX_NUCLEO_TCPI_Private_macro TCPI Private macro
  * @{
  */
#if defined(_RTOS)

/* Used to namage the I2C access */
#define LOCK_I2C_RESOURCE()    osSemaphoreWait(sem_i2c_res_id, osWaitForever)
#define UNLOCK_I2C_RESOURCE()  osSemaphoreRelease(sem_i2c_res_id)

#else /* !_RTOS */

/* Used to namage the I2C access */
#define LOCK_I2C_RESOURCE()          HAL_NVIC_DisableIRQ(ALERT_PORT0_EXTI_IRQn);
#define UNLOCK_I2C_RESOURCE()        HAL_NVIC_EnableIRQ(ALERT_PORT0_EXTI_IRQn);

#endif /* _RTOS */

#if defined(_TRACE)
#define POWER_DEBUG(__MSG__,__SIZE__)   USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0,__MSG__,__SIZE__);
#else
#define POWER_DEBUG(__MSG__,__SIZE__)
#endif /* _TRACE */

/**
  * @}
  */

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32F0XX_NUCLEO_TCPI_Private_Variables TCPI Private variables
  * @{
  */
GPIO_TypeDef*  VBUS_PORT[USBPD_PORT_COUNT] = {
  VBUS_GPIO_PORT0,
#if USBPD_PORT_COUNT==2
  VBUS_GPIO_PORT1
#endif
};
const uint16_t VBUS_PIN[USBPD_PORT_COUNT] = {
  VBUS_GPIO_PIN_PORT0, 
#if USBPD_PORT_COUNT==2
  VBUS_GPIO_PIN_PORT1
#endif
};
/* Local variable to store port status */
static t_port_mapping port_list[USBPD_PORT_COUNT];

/* Handle to drive i2c communication*/
I2C_HandleTypeDef hI2cHandle;

#if defined(_RTOS)
/* Mutex definition */
osSemaphoreDef(sem_i2c_res);
osSemaphoreId sem_i2c_res_id;
osSemaphoreDef(sem_i2c_cplt);
osSemaphoreId sem_i2c_cplt_id;
#endif /* _RTOS */

TCPC_DrvTypeDef* DevicesDrivers[USBPD_PORT_COUNT] = {
#ifdef TCPC_FL7101
  &FL7101_TCPC_drv,
#if USBPD_PORT_COUNT == 2
  &FL7101_TCPC_drv,
#endif /*USBPD_PORT_COUNT == 2*/
#else
#if defined(_TCPM_VP230)
  &vp230_tcpc_drv,
#elif defined(_TCPM_FUSB302)
  &fusb302_tcpc_drv,
#else
  &fusb305_tcpc_drv,
#endif /* _TCPM_VP230 */
#if USBPD_PORT_COUNT == 2
  &fusb305_tcpc_drv,
#endif /*USBPD_PORT_COUNT == 2*/
#endif /* TCPC_FL7101 */
};

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
#ifdef HAL_I2C_MODULE_ENABLED
/** @defgroup STM32F0XX_NUCLEO_TCPI_Private_Functions TCPI Private Functions
  * @{
  */
static uint32_t  TCPI_Init(I2C_HandleTypeDef *hi2c);
static uint32_t  TCPI_DeInit(I2C_HandleTypeDef *hi2c);

/**
  * @}
  */
#endif /* HAL_I2C_MODULE_ENABLED */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup STM32F0XX_NUCLEO_TCPI_Functions TCPI Exported Functions
  * @{
  */

/** @defgroup STM32F0XX_NUCLEO_TCPI_Functions_Grp1 TCPI Exported Functions for I2C Link
  * @{
  */

void USBPD_TCPI_AlertInit(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  ALERT_PORT0_GPIO_CLK_ENABLE();
#if USBPD_PORT_COUNT==2
  ALERT_PORT1_GPIO_CLK_ENABLE();
#endif /* USBPD_PORT_COUNT==2 */

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = ALERT_PORT0_GPIO_PIN;
  HAL_GPIO_Init(ALERT_PORT0_GPIO_PORT, &GPIO_InitStructure);
#if USBPD_PORT_COUNT==2
  GPIO_InitStructure.Pin = ALERT_PORT1_GPIO_PIN;
  HAL_GPIO_Init(ALERT_PORT1_GPIO_PORT, &GPIO_InitStructure);
#endif /* USBPD_PORT_COUNT==2 */
  
  /* Enable and set EXTI line 2_3 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(ALERT_PORT0_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(ALERT_PORT0_EXTI_IRQn);
#if USBPD_PORT_COUNT==2
  /* Enable and set EXTI line 0_1 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(ALERT_PORT1_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(ALERT_PORT1_EXTI_IRQn);
#endif /* USBPD_PORT_COUNT==2 */
}

USBPD_StatusTypeDef USBPD_TCPCI_Init(void)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

  for(uint32_t i= 0 ; i< USBPD_PORT_COUNT; i++)
  {
    port_list[i].i2c_addr = 0;
  }

  if( 0 != TCPI_Init(&hI2cHandle))
  {
    _retr = USBPD_FAIL;
  }

#ifdef TCPC_FL7101
  Initial_FL7101();
#endif /* TCPC_FL7101 */
  
  port_list[USBPD_PORT_0].i2c_addr = I2C_SLAVE_ADDRESS_PORT0;
#if USBPD_PORT_COUNT == 2
  port_list[USBPD_PORT_1].i2c_addr = I2C_SLAVE_ADDRESS_PORT1;
#endif /* USBPD_PORT_COUNT == 2 */

#if defined(_RTOS)
  /* Init the I2C semaphores */
  if((sem_i2c_res_id = osSemaphoreCreate(osSemaphore(sem_i2c_res), 1)) == 0)
  {
    _retr = USBPD_FAIL;
  }

  if((sem_i2c_cplt_id = osSemaphoreCreate(osSemaphore(sem_i2c_cplt), 1)) == 0)
  {
    _retr = USBPD_FAIL;
  }

  /* take the mutex_I2C_CPLT semaphores */
  osSemaphoreWait(sem_i2c_cplt_id, osWaitForever);
#else
  USBPD_TCPI_AlertInit();
#endif /* _RTOS */

  return _retr;
}

USBPD_StatusTypeDef USBPD_TCPCI_DeInit(void)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

  for(uint32_t i= 0 ; i< USBPD_PORT_COUNT; i++)
  {
    port_list[i].i2c_addr = 0;
  }

  if( 0 != TCPI_DeInit(&hI2cHandle))
  {
    _retr = USBPD_FAIL;
  }

#if defined(_RTOS)
  /* Delete the mutex */
  if( osOK != osSemaphoreDelete(sem_i2c_res_id)) _retr = USBPD_FAIL;
  if( osOK != osSemaphoreDelete(sem_i2c_cplt_id)) _retr = USBPD_FAIL;
#endif /* _RTOS */
  
  return _retr;
}

USBPD_StatusTypeDef USBPD_TCPCI_GetDevicesDrivers(uint8_t PortNum, TCPC_DrvTypeDef **TCPC_Driver)
{
  *TCPC_Driver = DevicesDrivers[PortNum];

  return USBPD_OK;
}

USBPD_StatusTypeDef USBPD_TCPCI_WriteRegister(uint8_t port, uint8_t RegisterId, uint8_t *prtData, uint8_t datasize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
#ifdef TCPC_FL7101
  uint16_t w_reg;
#endif /* TCPC_FL7101 */

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();

#ifdef TCPC_FL7101
  if (port == 0)
    w_reg = RegisterId;
  else
    w_reg = 0x1000 | RegisterId;
#endif /* TCPC_FL7101 */
  
#ifdef TCPC_FL7101
  if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[port].i2c_addr, w_reg, 2, prtData, datasize, 100 ))
#else
  if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[port].i2c_addr, RegisterId, 1,
                                     prtData, datasize, 100 ))
#endif /* TCPC_FL7101 */
  {
    _retr = USBPD_FAIL;
  }
  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
  
  return _retr;
}

USBPD_StatusTypeDef USBPD_TCPCI_SendTransmitBuffer(uint32_t Port, uint8_t RegisterId, uint8_t TransmitByteCount, uint16_t Header, uint8_t *pData)
{
#if defined(_TCPM_FUSB302)
#else
  USBPD_StatusTypeDef _retr = USBPD_OK;
#ifdef TCPC_FL7101
  uint16_t w_reg;
#endif /* TCPC_FL7101 */

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();

#ifdef TCPC_FL7101
  if (Port == 0)
    w_reg = RegisterId;
  else
    w_reg = 0x1000 | RegisterId;
#endif /* TCPC_FL7101 */

#ifdef TCPC_FL7101
  if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                   &TransmitByteCount, 1, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }

  if (Port == 0)
    w_reg = TCPC_REG_TX_HEADER;
  else
    w_reg = 0x1000 | TCPC_REG_TX_HEADER;
    
  if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                                 (uint8_t*)&Header, 2, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }

  if (Port == 0)
    w_reg = TCPC_REG_TX_DATA;
  else
    w_reg = 0x1000 | TCPC_REG_TX_DATA;

  if (TransmitByteCount > 2)
  {
    if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                     pData, (TransmitByteCount - 2), 100 ))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
  }
#else
    if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, RegisterId, 1,
                                       &TransmitByteCount, 1, 100 ))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
    
    if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, TCPC_REG_TX_HEADER, 1,
                                       (uint8_t*)&Header, 2, 100 ))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
    
    if (TransmitByteCount > 2)
    {
      if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[Port].i2c_addr, TCPC_REG_TX_DATA, 1,
                                         pData, (TransmitByteCount - 2), 100 ))
      {
        _retr = USBPD_FAIL;
        goto exit;
      }
    }
#endif /* TCPC_FL7101 */

exit :  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
#endif /* _TCPM_FUSB302 */
  
  return _retr;
}

USBPD_StatusTypeDef USBPD_TCPCI_ReceiveBuffer(uint32_t Port, uint8_t RegisterId, uint8_t *Buffer, uint8_t *SOPType)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
#if defined(_TCPM_FUSB302)
#else
  uint8_t count = 0;
#ifdef TCPC_FL7101
  uint16_t w_reg;
#endif /* TCPC_FL7101 */

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();

#ifdef TCPC_FL7101
  if (Port == 0)
    w_reg = RegisterId;
  else
    w_reg = 0x1000 | RegisterId;
#endif /* TCPC_FL7101 */
  
#ifdef TCPC_FL7101
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                   &count, 1, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }  
  
  if (Port == 0)
    w_reg = TCPC_REG_RX_BUFFER_FRAME_TYPE;
  else
    w_reg = 0x1000 |  TCPC_REG_RX_BUFFER_FRAME_TYPE;
  
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                   SOPType, 1, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  if (count != 0)
  {
    if (Port == 0)
      w_reg = TCPC_REG_RX_HEADER;
    else  
      w_reg = 0x1000 | TCPC_REG_RX_HEADER;      
    if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2,
                    Buffer, (count - 1), 100 ))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
  }
#else
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, RegisterId, 1,
                                     &count, 1, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, TCPC_REG_RX_BUFFER_FRAME_TYPE, 1,
                                     SOPType, 1, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  if (count != 0)
  {
    if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, TCPC_REG_RX_HEADER, 1,
                                  Buffer, (count - 1), 100 ))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
  }
#endif /* TCPC_FL7101 */
exit :  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
#endif /* _TCPM_FUSB302 */
  
  return _retr;
}

#if defined(_TCPM_FUSB302)
USBPD_StatusTypeDef USBPD_TCPCI_SearchAddress(uint8_t Port, uint8_t RegisterId, uint8_t *prtData, uint8_t datasize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();

#ifdef TCPC_FL7101
  uint16_t w_reg;
  if (Port == 0)
    w_reg = RegisterId;
  else  //Port 1 start from 0x1000
    w_reg = 0x1000 | RegisterId;


  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2, prtData, datasize, 100 ))
  {
    _retr = USBPD_FAIL;
  }
#else
  uint16_t list_addr[4] = {
                        FUSB302_I2C_SLAVE_ADDR,
                        FUSB302_I2C_SLAVE_ADDR_B01,     /* FUSB302B01MPX */
                        FUSB302_I2C_SLAVE_ADDR_B10,     /* FUSB302B10MPX */
                        FUSB302_I2C_SLAVE_ADDR_B11,     /* FUSB302B11MPX */
  };
  for (uint32_t index = 0; index < 4; index++)
  {
    port_list[Port].i2c_addr = list_addr[index];
    if(HAL_OK == HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, RegisterId, 1,
                                       prtData, datasize, 100 ))
    {
      break;
    }
    /* Read the DeviceID register to get the chip version */
    if ((*prtData == FUSB302_DEVID_302B)
        || (*prtData == FUSB302_DEVID_302A))
    {
      break;
    }
  }
#endif

  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();

  return _retr;
}
#endif /* _TCPM_FUSB302 */

USBPD_StatusTypeDef USBPD_TCPCI_ReadRegister(uint8_t Port, uint8_t RegisterId, uint8_t *prtData, uint8_t datasize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
#ifdef TCPC_FL7101
  uint16_t w_reg;
#endif /* TCPC_FL7101 */

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();

#ifdef TCPC_FL7101
  if (Port == 0)
    w_reg = RegisterId;
  else
    w_reg = 0x1000 | RegisterId;
#endif /* TCPC_FL7101 */

#ifdef TCPC_FL7101
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, w_reg, 2, prtData, datasize, 100 ))
#else
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, RegisterId, 1,
                                     prtData, datasize, 100 ))
#endif
  {
    _retr = USBPD_FAIL;
  }

  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();

  return _retr;
}

void USBPD_TCPCI_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @}
  */

/** @defgroup STM32F0XX_NUCLEO_TCPI_Functions_Grp2 TCPI Exported function for PWR
  * @{
  */

USBPD_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage)
{
#ifdef TCPC_FL7101
  PV2105_Set_Output_Voltage(Port, Voltage);
#endif /* TCPC_FL7101 */
  return USBPD_OK;
}

uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum)
{
  return USBPD_TCPM_VBUS_GetVoltage(PortNum);
}

int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum)
{
  return 0;
}

USBPD_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState state, CCxPin_TypeDef Cc, uint32_t VconnState, USBPD_PortPowerRole_TypeDef role)
{
  USBPD_StatusTypeDef status;
  if(USBPD_ENABLE == state)
  {
    POWER_DEBUG("VBUS ON", 7);
    if (USBPD_TRUE == VconnState)
    {
      POWER_DEBUG("VCONN ON", 8);
      /* Enable VConn */
      USBPD_TCPM_set_vconn(PortNum, USBPD_ENABLE);
    }
    status = USBPD_TCPM_VBUS_Enable(PortNum);
  }
  else
  {
    POWER_DEBUG("VBUS OFF", 8);
    if (USBPD_TRUE == VconnState)
    {
      POWER_DEBUG("VCONN OFF", 9);
      USBPD_TCPM_set_vconn(PortNum, USBPD_DISABLE);
    }
    status = USBPD_TCPM_VBUS_Disable(PortNum);
  }
  return status;
}

USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum)
{
  return ((USBPD_OK == USBPD_TCPM_VBUS_IsVsafe5V(PortNum)) ? USBPD_ENABLE : USBPD_DISABLE);
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef HAL_I2C_MODULE_ENABLED
/** @addtogroup STM32F0XX_NUCLEO_TCPI_Private_Functions
  * @{
  */
/**
  * @brief  Initialize TCPI 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
static uint32_t TCPI_Init(I2C_HandleTypeDef *hi2c)
{
  uint32_t _retr = 0;

  /*##-1- Configure the I2C peripheral ######################################*/
  hi2c->Instance             = I2Cx;
  hi2c->Init.Timing          = I2C_TIMING;
  hi2c->Init.OwnAddress1     = 0;
  hi2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2     = 0;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  
  if(HAL_OK != HAL_I2C_Init(hi2c))
  {
    _retr = 1;
  }
  
  /* Enable the Analog I2C Filter */
  if(HAL_OK != HAL_I2CEx_ConfigAnalogFilter(hi2c,I2C_ANALOGFILTER_ENABLE))
  {
    _retr = 1;
  }
  
  return _retr;
}

/**
  * @brief  Un-Initialize TCPCI 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
static uint32_t TCPI_DeInit(I2C_HandleTypeDef *hi2c)
{
  uint32_t _retr = 0;

  if(HAL_OK != HAL_I2C_DeInit(hi2c))
  {
    _retr = 1;
  }

  return _retr;
}


/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();

  /*##-3- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

/**
  * @}
  */

#endif /* HAL_I2C_MODULE_ENABLED */

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

