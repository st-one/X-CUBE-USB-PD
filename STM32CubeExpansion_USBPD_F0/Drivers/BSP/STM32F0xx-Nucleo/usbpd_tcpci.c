/**
  ******************************************************************************
  * @file    usbpd_tcpci.c
  * @author  MCD Application Team
  * @version V0.0.3
  * @date    12-April-2016
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
#include "usbpd_def.h"
#include "usbpd_conf.h"
#include "tcpc.h"
#include "usbpd_tcpci.h"
#include "cmsis_os.h"
#include "stm32f0xx_nucleo.h"
#include "string.h" 
#include "stm32f0xx_hal_i2c.h"

/* Private typedef -----------------------------------------------------------*/
#ifdef USBPD_TCPC
typedef enum {
   TCPCI_STATE_INIT,
   TCPCI_STATE_WAIT_REGID,
   TCPCI_STATE_READ_TXCOUNT,
   TCPCI_STATE_CHANGE_FOR_READ,
   TCPCI_STATE_READ_DATAEND,
   TCPCI_STATE_NACK,
   
   TCPCI_STATE_WRITE_RXCOUNT, 
   
   TCPCI_ERROR_SEQ_RECEIVE,
   TCPCI_ERROR_SEQ_TRANSMIT,
   TCPCI_ERROR_SEQ_UNKOWN,
   TCPCI_ERROR_SEQ_TXCOUNT

} TCPC_enum;
#endif


typedef struct{
#ifndef USBPD_TCPC
  uint16_t i2c_addr;
  // callback function to store 
#else
  TCPC_enum error;
  uint8_t direction;
  USBPD_TCPC_RegisterId registerid;
  TCPC_enum state;
  uint8_t databuffer[100];
#endif  
} t_port_mapping;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Wait and check the I2C operation complete */
#define CHECK_I2C_STATUS() do {                                   \
              if (osSemaphoreWait(sem_i2c_cplt_id, 1000) == osOK) \
              {                                                   \
                switch(HAL_I2C_GetError(&hI2cHandle))             \
                {                                                 \
                  case HAL_I2C_ERROR_NONE:                        \
                  break;                                          \
                  case HAL_I2C_ERROR_AF:                          \
                  /* Maybe need to add a retry mechanisme*/       \
                   default :                                      \
                  _retr = USBPD_FAIL;                             \
                  goto exit;                                      \
                  break;                                          \
                }                                                 \
              } else {_retr = USBPD_FAIL; goto exit;}             \
            }while(0);

#define CHECK_I2C_STATUS_LISTEN() do {                                   \
              if (osSemaphoreWait(sem_i2c_cplt_id, osWaitForever) == osOK) \
              {                                                   \
                switch(HAL_I2C_GetError(&hI2cHandle))             \
                {                                                 \
                  case HAL_I2C_ERROR_NONE:                        \
                  break;                                          \
                  case HAL_I2C_ERROR_AF:                          \
                  /* Maybe need to add a retry mechanisme*/       \
                   default :                                      \
                  _retr = USBPD_FAIL;                             \
                  goto exit;                                      \
                  break;                                          \
                }                                                 \
              } else {_retr = USBPD_FAIL; goto exit;}             \
            }while(0);

/* Free the mutex to indicate the operation complete */              
#define OPERATION_COMPLETE()    osSemaphoreRelease(sem_i2c_cplt_id)
              
#ifndef USBPD_TCPC
/* Used to namage the I2C access */                
#define LOCK_I2C_RESOURCE()    osSemaphoreWait(sem_i2c_res_id, osWaitForever)
#define UNLOCK_I2C_RESOURCE()  osSemaphoreRelease(sem_i2c_res_id)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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

/* Mutex definition */
#ifndef USBPD_TCPC
osSemaphoreDef(sem_i2c_res);
osSemaphoreId sem_i2c_res_id;
#endif
osSemaphoreDef(sem_i2c_cplt);
osSemaphoreId sem_i2c_cplt_id;

#ifdef USBPD_TCPC
#define __DEBUG_TCPCI
#if defined(__DEBUG_TCPCI)
__IO USBPD_TCPC_RegisterId RegID[USBPD_PORT_COUNT][256];
__IO uint8_t IndexRegID[USBPD_PORT_COUNT];
__IO TCPC_enum             TCPCstate[USBPD_PORT_COUNT][256];
__IO uint8_t IndexState[USBPD_PORT_COUNT];
__IO TCPC_enum            TCPCerror[USBPD_PORT_COUNT][256];
__IO uint8_t IndexError[USBPD_PORT_COUNT];

#define __DEBUG_TRACE_STATE(HANDLE, __PORT_)  do {                                \
            if (RegID[(__PORT_)][IndexRegID[(__PORT_)]-1] != HANDLE.registerid)   \
            {                                                                     \
              RegID[(__PORT_)][IndexRegID[(__PORT_)]++] =   HANDLE.registerid;    \
              IndexRegID[(__PORT_)] = IndexRegID[(__PORT_)] % 256;                \
            }                                                                     \
            if (TCPCstate[(__PORT_)][IndexState[(__PORT_)]-1] != HANDLE.state)    \
            {                                                                     \
              TCPCstate[(__PORT_)][IndexState[(__PORT_)]++] =   HANDLE.state;     \
              IndexState[(__PORT_)] = IndexState[(__PORT_)] % 256;                \
            }                                                                     \
            if (TCPCerror[(__PORT_)][IndexError[(__PORT_)]-1] != HANDLE.error)    \
            {                                                                     \
              TCPCerror[(__PORT_)][IndexError[(__PORT_)]++] =   HANDLE.error;     \
              IndexError[(__PORT_)] = IndexError[(__PORT_)] % 256;                \
            }                                                                     \
           } while(0)
#else
#define __DEBUG_TRACE_STATE(HANDLE, __PORT_)
#endif /* __DEBUG_TCPCI */
#endif /* USBPD_TCPC */

/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32F1XX_NUCLEO_TCPI_Functions TCPI Functions
  * @{
  */ 

/**
  * @brief  Initialize TCPI Alert
  * @param  None
  * @retval 0 if OK 
  */
void BSP_TCPI_AlertInit(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  ALERT_GPIO_CLK_ENABLE();

#ifdef USBPD_TCPC
  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = ALERT_GPIO_PIN;
  HAL_GPIO_Init(ALERT_GPIO_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_SET);
#else /* TCPM */
  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
#if USBPD_PORT_COUNT==1
  GPIO_InitStructure.Pin = ALERT_GPIO_PIN;
#else
  GPIO_InitStructure.Pin = ALERT_GPIO_PIN | ALERT_PORT1_GPIO_PIN;
#endif
  HAL_GPIO_Init(ALERT_GPIO_PORT, &GPIO_InitStructure);
  
  /* Enable and set EXTI line 2_3 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(ALERT_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(ALERT_EXTI_IRQn);
#if USBPD_PORT_COUNT==2
  /* Enable and set EXTI line 0_1 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(ALERT_PORT1_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(ALERT_PORT1_EXTI_IRQn);
#endif
#endif  
}

#ifdef USBPD_TCPC
/**
  * @brief  ALERTE ON
  * @param  None
  * @retval 0 if OK 
  */
void BSP_TCPI_AlertON(void)
{
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  ALERTE ON
  * @param  None
  * @retval 0 if OK 
  */
void BSP_TCPI_AlertOFF(void)
{
  HAL_GPIO_WritePin(ALERT_GPIO_PORT, ALERT_GPIO_PIN, GPIO_PIN_SET);
}
#endif

#if 0
/**
  * @brief  Initialize TCPI VBUS Pin
  * @param  None
  * @retval 0 if OK 
  */
void BSP_TCPI_VBUS_Init(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  VBUS_GPIO_PORT0_CLK_ENABLE();
#if USBPD_PORT_COUNT==2
  VBUS_GPIO_PORT1_CLK_ENABLE();
#endif
  
  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = VBUS_GPIO_PIN_PORT0;
  HAL_GPIO_Init(VBUS_GPIO_PORT0, &GPIO_InitStructure);
#if USBPD_PORT_COUNT==2
  GPIO_InitStructure.Pin = VBUS_GPIO_PIN_PORT1;
  HAL_GPIO_Init(VBUS_GPIO_PORT1, &GPIO_InitStructure);
#endif
}

/**
  * @brief  VBUS ON
  * @param  Port Port of VBUS to enable
  * @retval 0 if OK 
  */
void BSP_TCPI_VBUS_ON(uint32_t Port)
{
  HAL_GPIO_WritePin(VBUS_PORT[Port], VBUS_PIN[Port], GPIO_PIN_SET);
}

/**
  * @brief  VBUS OFF
  * @param  Port of VBUS to disable
  * @retval 0 if OK 
  */
void BSP_TCPI_VBUS_OFF(uint32_t Port)
{
  HAL_GPIO_WritePin(VBUS_PORT[Port], VBUS_PIN[Port], GPIO_PIN_RESET);
}
#endif

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  Port        The port handle.
  * @param  Voltage     voltage value to be set (in mV).
  * @retval None
  */
void BSP_USBPD_SetVoltage(uint32_t Port, uint32_t Voltage)
{
//  switch (Voltage)
//  {
//  case 12000:
//    status = tcpc_config[Port].drv->set_voltage(Port, );
//    break;
//  case 5000:
//  default:
//    HAL_GPIO_WritePin(USBPDM1_POWSELs[0+off].GPIOx, USBPDM1_POWSELs[0+off].GPIO_Pin, GPIO_PIN_RESET);
//    break;
//  }
}

/**
  * @brief  Get the voltage level on a specified port.
  * @param  Port The port handle.
  * @retval The voltage value 
  */
uint16_t BSP_USBPD_GetVoltage(uint32_t Port)
{
  return 0;
}

/**
  * @brief  Get the current level on a specified port.
  * @param  Port The port handle.
  * @retval The current value 
  */
uint16_t BSP_USBPD_GetCurrent(uint32_t Port)
{
  return 0;
}

#ifdef HAL_I2C_MODULE_ENABLED
/**
  * @brief  Initialize TCPI 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
uint32_t BSP_TCPI_Init(I2C_HandleTypeDef *hi2c)
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
uint32_t BSP_TCPI_DeInit(I2C_HandleTypeDef *hi2c)
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

#ifdef I2C_CLOCKSOURCE_NA
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
#endif
  
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
    
  /*##-4- Configure the NVIC for I2C ########################################*/   
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(I2Cx_IRQn);
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
  
  /*##-3- Disable the NVIC for I2C ##########################################*/
  HAL_NVIC_DisableIRQ(I2Cx_IRQn);
}
#endif
/**
  * @}
  */

/**
  * @brief  Initialize TCPCI 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_Init(void)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

#ifndef USBPD_TCPC  
  for(uint32_t i= 0 ; i< USBPD_PORT_COUNT; i++)
  {
    port_list[i].i2c_addr = 0;
  }
#endif
  
  if( 0 != BSP_TCPI_Init(&hI2cHandle))
  {
    _retr = USBPD_FAIL;
  }
  
  BSP_TCPI_AlertInit();
  
#if 0
  BSP_TCPI_VBUS_Init();
#endif
  
#ifndef USBPD_TCPC  
  /* Init the I2C semaphores */
  if((sem_i2c_res_id = osSemaphoreCreate(osSemaphore(sem_i2c_res), 1)) == 0)
  {
    _retr = USBPD_FAIL;
  }
#endif
  
  if((sem_i2c_cplt_id = osSemaphoreCreate(osSemaphore(sem_i2c_cplt), 1)) == 0)
  {
    _retr = USBPD_FAIL;
  }
  
  /* take the mutex_I2C_CPLT semaphores */
  osSemaphoreWait(sem_i2c_cplt_id, osWaitForever);
  
  return _retr;
}

/**
  * @brief  Un-Initialize TCPCI 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_DeInit(void)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

#ifndef USBPD_TCPC  
  for(uint32_t i= 0 ; i< USBPD_PORT_COUNT; i++)
  {
    port_list[i].i2c_addr = 0;
  }
#endif
  
  if( 0 != BSP_TCPI_DeInit(&hI2cHandle))
  {
    _retr = USBPD_FAIL;
  }

  
  /* Delete the mutex */
#ifndef USBPD_TCPC
  if( osOK != osSemaphoreDelete(sem_i2c_res_id)) _retr = USBPD_FAIL;
#endif
  if( osOK != osSemaphoreDelete(sem_i2c_cplt_id)) _retr = USBPD_FAIL;
  
  return _retr;
}

#ifndef USBPD_TCPC
/**
  * @brief  TCPC register port 
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_RegisterPort(uint8_t port, uint32_t i2c_addr)
{
  port_list[port].i2c_addr = i2c_addr;

  return USBPD_OK;
}

/**
  * @brief  TCPCI write register  
  * @param  portnum: Index of current used port
  * @param  registerId : register Id
  * @param  ptrData : data pointer
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_WriteRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
  
  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();
  
#if 1
  if(HAL_OK != HAL_I2C_Mem_Write(&hI2cHandle, port_list[port].i2c_addr, registerId, 1,
                                     prtData, datasize, 100 ))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
#else 
  /* peut etre prevoir un mechanisme de retry */
  
  /* Send the registerid */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[port].i2c_addr, &registerId, 1, I2C_NEXT_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  /* Send the datas */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[port].i2c_addr, prtData, datasize, I2C_LAST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
#endif
  
exit:
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
  
  return _retr;
}

/**
  * @brief  TCPCI sends Transmitter buffer message  
  * @param  Port Index of current used port
  * @param  RegisterId register Id
  * @param  pData data pointer
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_SendTransmitBuffer(uint32_t Port, uint8_t RegisterId, uint8_t TransmitByteCount, uint16_t Header, uint8_t *pData)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
  
  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();
  
#if 1
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
#else 
  /* Send the registerid */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, &RegisterId, 1, I2C_NEXT_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  /* Send the Register ID */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, &TransmitByteCount, 1, I2C_NEXT_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  if (TransmitByteCount > 2)
  {
    /* Send the Header bytes */
    if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, (uint8_t*)&Header, 2, I2C_NEXT_FRAME))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
    
    /* check the transmit status */
    CHECK_I2C_STATUS();
    
    /* Send the datas */
    if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, pData, (TransmitByteCount - 2), I2C_LAST_FRAME))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }
    
    /* check the transmit status */
    CHECK_I2C_STATUS();
  }
  else
  {
    /* Send the Header bytes */
    if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, (uint8_t*)&Header, 2, I2C_LAST_FRAME))
    {
      _retr = USBPD_FAIL;
      goto exit;
    }

    /* check the transmit status */
    CHECK_I2C_STATUS();
  }
#endif  
exit :  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
  
  return _retr;
}

/**
  * @brief  TCPCI receives buffer message  
  * @param  Port Index of current used port
  * @param  RegisterId register Id
  * @param  pData data pointer
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_ReceiveBuffer(uint32_t Port, uint8_t registerId, uint8_t *Buffer, uint8_t *SOPType)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
  uint8_t count = 0;
  
  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();
  
#if 1
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[Port].i2c_addr, registerId, 1,
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
#else
  /* Send the registerid */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[Port].i2c_addr, &registerId, 1, I2C_FIRST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  /* Receive the RX COUNT */
  if(HAL_OK != HAL_I2C_Master_Sequential_Receive_IT(&hI2cHandle, port_list[Port].i2c_addr, &count, 1, I2C_NEXT_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  /* Get the buffer */
  if(HAL_OK != HAL_I2C_Master_Sequential_Receive_IT(&hI2cHandle, port_list[Port].i2c_addr, SOPType, 1, I2C_NEXT_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
  /* Get the buffer */
  if(HAL_OK != HAL_I2C_Master_Sequential_Receive_IT(&hI2cHandle, port_list[Port].i2c_addr, Buffer, (count - 1), I2C_LAST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* check the transmit status */
  CHECK_I2C_STATUS();
  
#endif  
exit :  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();
  
  return _retr;
}

/**
  * @brief  TCPCI read register  
  * @param  portnum Index of current used port
  * @param  registerId register Id
  * @param  ptrData data pointer
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_ReadRegister(uint8_t port, uint8_t registerId, uint8_t *prtData, uint8_t datasize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();
  
#if 1
  if(HAL_OK != HAL_I2C_Mem_Read(&hI2cHandle, port_list[port].i2c_addr, registerId, 1,
                                     prtData, datasize, 100 ))
  {
    _retr = USBPD_FAIL;
  }
#else
  /* Send the registerid */
  if(HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(&hI2cHandle, port_list[port].i2c_addr, &registerId, 1, I2C_FIRST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  
  /* Wait the I2C complete */
  CHECK_I2C_STATUS();
  
  /* Send the datas */
  if(HAL_OK != HAL_I2C_Master_Sequential_Receive_IT(&hI2cHandle, port_list[port].i2c_addr, prtData, datasize, I2C_LAST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  /* check the transmit status */
  CHECK_I2C_STATUS();

exit:
  if (HAL_I2C_GetError(&hI2cHandle) == HAL_I2C_ERROR_ARLO) _retr = USBPD_FAIL;
#endif

  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();

  return _retr;
}

/**
  * @brief  TCPCI write buffer  
  * @param  Port     Index of current used port
  * @param  ptrData     data pointer
  * @param  DataSize    Data size of the buffer to send
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_WriteBuffer(uint8_t Port, uint8_t *prtData, uint16_t DataSize)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;

  /* Reserve the I2C ressource */
  LOCK_I2C_RESOURCE();
  
  /* Send the datas */
  if(HAL_OK != HAL_I2C_Master_Sequential_Receive_IT(&hI2cHandle, port_list[Port].i2c_addr, prtData, DataSize, I2C_FIRST_AND_LAST_FRAME))
  {
    _retr = USBPD_FAIL;
    goto exit;
  }
  /* check the transmit status */
  CHECK_I2C_STATUS();

exit:
  if (HAL_I2C_GetError(&hI2cHandle) == HAL_I2C_ERROR_ARLO) _retr = USBPD_FAIL;
  
  /* free the I2C ressource */
  UNLOCK_I2C_RESOURCE();

  return _retr;
}

/**
  * @brief  TCPCI delay  
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void USBPD_TCPCI_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/*************************************************************************/
/* I2C callback implementation                                           */
/*************************************************************************/
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  OPERATION_COMPLETE();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  OPERATION_COMPLETE();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  OPERATION_COMPLETE();
}
#else
extern uint8_t TCPC_GetDataSize(uint8_t reg);
extern uint8_t *TCPC_GetDataSizePtr(uint8_t reg, uint8_t *datasize);

/**
  * @brief  TCPC Get the register id
  * @param  portnum: Index of current used port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_ListenPort(uint8_t port, uint8_t *direction, uint8_t *registerid, uint8_t **ptr)
{
  USBPD_StatusTypeDef _retr = USBPD_OK;
 
  port_list[0].state = TCPCI_STATE_INIT;
  
  /* Put I2C in Listen */  
  if(HAL_OK != HAL_I2C_EnableListen_IT(&hI2cHandle))
  {
    _retr = USBPD_FAIL;
  }
  
  CHECK_I2C_STATUS_LISTEN();
  
  *direction = port_list[0].direction;
  *registerid = port_list[0].registerid;
  *ptr = port_list[0].databuffer;

  __DEBUG_TRACE_STATE(port_list[0], 0);

exit:  
  return _retr;
}



/*************************************************************************/
/* I2C callback implementation                                           */
/*************************************************************************/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  port_list[0].registerid = 0xFF;
  __DEBUG_TRACE_STATE(port_list[0], 0);
  OPERATION_COMPLETE(); 
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
//  port_list[0].registerid = 0xFF;
//  OPERATION_COMPLETE(); 
}
/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  port_list[0].direction = TransferDirection;
  if(I2C_DIRECTION_TRANSMIT ==  TransferDirection) //&& (port_list[0].state == TCPCI_STATE_INIT))
  {
    port_list[0].state = TCPCI_STATE_WAIT_REGID;
    // register information is expected
    if(HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, (uint8_t *)&port_list[0].registerid, 1, I2C_FIRST_FRAME))
    {
      // erreur detected
      port_list[0].error = TCPCI_ERROR_SEQ_RECEIVE;
    }
  }
  else  
  {  
    uint8_t *_dataptr = NULL;
    uint8_t _datasize =0;
    
    /* Direction change, abort the current Slave RX (workaround !!!) */
    hi2c->State = HAL_I2C_STATE_LISTEN;
    
    /* get information about how many data to send */
    if(port_list[0].registerid == TCPC_REG_RX_BYTE_COUNT)
    {
      _dataptr = TCPC_GetDataSizePtr(port_list[0].registerid, &_datasize);
      port_list[0].state = TCPCI_STATE_WRITE_RXCOUNT; 
      // get data pointer 
      if(HAL_OK != HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, _dataptr , _datasize, I2C_NEXT_FRAME))
      {
        // erreur detected
        port_list[0].error = TCPCI_ERROR_SEQ_TRANSMIT;
      }
    }
    else if((_dataptr = TCPC_GetDataSizePtr(port_list[0].registerid, &_datasize)) != NULL)
    {
      port_list[0].state = TCPCI_STATE_CHANGE_FOR_READ; 
      // get data pointer 
      if(HAL_OK != HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, _dataptr , _datasize, I2C_LAST_FRAME))
      {
        // erreur detected
        port_list[0].error = TCPCI_ERROR_SEQ_TRANSMIT;
      }
    }
    else
    {
        // NACK the unkown request 
        port_list[0].state = TCPCI_STATE_NACK; 
        __HAL_I2C_GENERATE_NACK(hi2c);
    }
  }
  __DEBUG_TRACE_STATE(port_list[0], 0);
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // The RX is complete 
  switch(port_list[0].state)
  {
  case TCPCI_STATE_WAIT_REGID :
    if(port_list[0].registerid == TCPC_REG_TX_BYTE_COUNT)
    {
      /* read the number of data exepected */
      port_list[0].state = TCPCI_STATE_READ_TXCOUNT;
      if(HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, port_list[0].databuffer, 1, I2C_NEXT_FRAME))
      {
        // erreur detected
        port_list[0].error = TCPCI_ERROR_SEQ_RECEIVE;
      }
    }
    else
    {
      __IO uint8_t _datasize;

      /* get information about how many data to send */
      if((_datasize =  TCPC_GetDataSize(port_list[0].registerid)) != 0x0u )
      {
        port_list[0].state = TCPCI_STATE_READ_DATAEND; 
        /* get data pointer  */
        if(HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, port_list[0].databuffer, _datasize, I2C_LAST_FRAME))
        {
          // erreur detected
          port_list[0].error = TCPCI_ERROR_SEQ_TRANSMIT;
        }
      }
      else
      {
        // NACK the unkown request 
        __HAL_I2C_GENERATE_NACK(hi2c);
        port_list[0].state = TCPCI_STATE_INIT; 
        port_list[0].error = TCPCI_ERROR_SEQ_UNKOWN;
        OPERATION_COMPLETE();
      }
    }
  break;
    
  case TCPCI_STATE_READ_TXCOUNT :
    if(HAL_OK != HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &port_list[0].databuffer[1], port_list[0].databuffer[0], I2C_LAST_FRAME))
    {
      // erreur detected
      __HAL_I2C_GENERATE_NACK(hi2c);
      port_list[0].state = TCPCI_STATE_INIT;
      port_list[0].error = TCPCI_ERROR_SEQ_TXCOUNT;
      OPERATION_COMPLETE();
    } else {
      port_list[0].state = TCPCI_STATE_READ_DATAEND;
    }
    break;

  case TCPCI_STATE_READ_DATAEND :
    // Communication closed 
    port_list[0].state = TCPCI_STATE_INIT;
    OPERATION_COMPLETE();
    break;
  }
  __DEBUG_TRACE_STATE(port_list[0], 0);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (port_list[0].state == TCPCI_STATE_WRITE_RXCOUNT)
  {
    uint8_t temp;
    uint8_t* pBuffer;
    pBuffer = TCPC_GetDataSizePtr(TCPC_REG_RX_DATA, &temp);
    // read the number of data exepected
    port_list[0].state = TCPCI_STATE_READ_DATAEND;
    if(HAL_OK != HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, pBuffer, temp, I2C_LAST_FRAME))
    {
      // erreur detected
      port_list[0].error = TCPCI_ERROR_SEQ_RECEIVE;
    }
  }
  else
  {
    port_list[0].state = TCPCI_STATE_INIT;
    /* Closed the communication */
    OPERATION_COMPLETE();
  }
  __DEBUG_TRACE_STATE(port_list[0], 0);
}

#endif
