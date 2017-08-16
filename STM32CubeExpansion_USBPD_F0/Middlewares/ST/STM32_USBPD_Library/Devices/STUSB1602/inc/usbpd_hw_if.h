/**
  ******************************************************************************
  * @file    usbpd_hw_if.h
  * @author  System Lab
  * @version V1.2.1
  * @date    24-Apr-2017
  * @brief   This file contains the headers of usbpd_hw_if.h.
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

#ifndef __STM32F072_USBPD_HW_IF_H_
#define __STM32F072_USBPD_HW_IF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_conf.h"
#include "usbpd_def.h"
#include "usbpd_porthandle.h"

/* Defines for STUSB16xx_EVAL (to be rewieved) -------------------------------*/
#define USBPD_ADCn		1


#define STM32_D_N_Pin                   GPIO_PIN_11
#define STM32_D_N_GPIO_Port             GPIOA
#define STM32_D_P_Pin                   GPIO_PIN_12
#define STM32_D_P_GPIO_Port             GPIOA
#define TMS_Pin                         GPIO_PIN_13
#define TMS_GPIO_Port                   GPIOA
#define TCK_Pin                         GPIO_PIN_14
#define TCK_GPIO_Port                   GPIOA

/* Exported typedef ----------------------------------------------------------*/
/**
  * @brief Mode how to access functions containing a list of actions  
  */
typedef enum 
{
  ACKNOWLEDGE = 0,
  REQUEST = 1
} USBPD_HRPRS_Mode_TypeDef;     


/* ADC variables and parameters */
#if (USBPD_PORT_COUNT==1)
#define ADCCONVERTEDVALUES_BUFFER_SIZE          6 /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
#elif (USBPD_PORT_COUNT==2)
#define ADCCONVERTEDVALUES_BUFFER_SIZE          10 /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
#endif

/* Exported define -----------------------------------------------------------*/
#define HW_IF_ADC				ADC1
 
#if (USBPD_PORT_COUNT == 1)
#define USBPD_ADCn		1
#endif

#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC1_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC1_RELEASE_RESET()

/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA                        DMA1_Channel1

#define TX_FREQ         ( 300000 )
#define BMC_TX_FREQ     ( 2 * TX_FREQ )
 
/* Exported constants --------------------------------------------------------*/
#define TX_BUFFER_LEN	                28	/*!< Size of Tx Buffer */

/* Exported macro ------------------------------------------------------------*/
#define DRP_PORT			GPIOC
#define DRP_PIN				GPIO_PIN_12
#define DRP_SET_PROVIDER(X)		HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_SET)
#define DRP_SET_CONSUMER(X)		HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_RESET)

#define USBPD_PORT_IsValid(__Port__) ((__Port__) < (USBPD_PORT_COUNT))


/**
 * 			    	R603+R604	 ADC->DR    		   386k
 * Vbus(mV) = Vadc ----------- = ------- * 3300(mV) * ------
 * 				      R604		  4096					56k
 * */
#define MVOLT(X)	(((uint32_t)(X*825*193)/7)>>12)

/**
 * 50Vs - VDDA/2 = Vadcin
 * */
#define MAMP(X)		( (int32_t)((X*3300)>>10)-6600 )

#define OTHER_CC(__CC__)                ( ((CCxPin_TypeDef)(__CC__))==CC1 ? CC2 : ( (((CCxPin_TypeDef)(__CC__))==CC2) ? CC1 : CCNONE ) ) 

#if (USBPD_PORT_COUNT == 1)
#define VBUS_INDEX(__PORT__)		5
#elif (USBPD_PORT_COUNT == 2)
#define VBUS_INDEX(__PORT__)		((__PORT__ == 0) ? 8 : 9 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define CC1_INDEX(__PORT__)		0
#elif (USBPD_PORT_COUNT == 2)
#define CC1_INDEX(__PORT__)		((__PORT__ == 0) ? 0 : 2 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define CC2_INDEX(__PORT__)		3
#elif (USBPD_PORT_COUNT == 2)
#define CC2_INDEX(__PORT__)		((__PORT__ == 0) ? 5 : 4 )
#endif

#define CAD_threshold_SNK_vRd_USB               MV2ADC(200)     /**< SNK vRd threshold = 0.2V  - USB default */
#define CAD_threshold_SNK_vRd_1_5A              MV2ADC(660)     /**< SNK vRd threshold = 0.66V  - 5V 1.5A */ 
#define CAD_threshold_SNK_vRd_3_0A              MV2ADC(1230)    /**< SNK vRd threshold = 1.23V  - 5V 3.0A */ 

#define CAD_threshold_SRC_vRa_5V_3A             MV2ADC(800)     /**< SRC vRa threshold = 0.8V  - 5V 3.0A */
#define CAD_threshold_SRC_vRd_5V_3A             MV2ADC(2600)    /**< SRC vRd threshold = 2.6V  - 5V 3.0A */
   
#define CAD_tPDDebounce_threshold               20              /**< tPDDebounce threshold = 20ms  */   
#define CAD_tCCDebounce_threshold               100             /**< tCCDebounce threshold = 100ms  */
   
#define CAD_threshold_VBus                      MV2ADC(500)     /**< Vbus Threshold **/
#define BIST_MAX_LENGTH                         30 //45ms (BIST_CARRIER_MODE_MS*300)/(TX_BUFFER_LEN*32)

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Called by interrupts */ 
void USBPD_DMA_PORT0_IRQHandler(void);
#if (USBPD_PORT_COUNT == 2)
void USBPD_DMA_PORT1_IRQHandler(void);
#endif
void USBPD_RX_PORT0_Interrupt_IRQHandler(void);
#if (USBPD_PORT_COUNT == 2)
void USBPD_RX_PORT1_Interrupt_IRQHandler(void);
#endif
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void);
#if (USBPD_PORT_COUNT == 2)
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void);
#endif
void USBPD_PE_PRL_TIMER_IRQHandler(void);


void USBPD_HW_IF_GlobalHwInit(void);
HAL_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role);

HAL_StatusTypeDef USBPD_HW_IF_PRS_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode);
HAL_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rd(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
HAL_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rp(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
HAL_StatusTypeDef USBPD_HW_IF_PRS_Vbus_OFF(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
HAL_StatusTypeDef USBPD_HW_IF_PRS_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);

HAL_StatusTypeDef USBPD_HW_IF_HR_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode);
HAL_StatusTypeDef USBPD_HW_IF_HR_CheckVbusVSafe0V(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
HAL_StatusTypeDef USBPD_HW_IF_HR_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);

void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState);
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio);
void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio);
void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio);


/* Private functions --------------------------------------------------------*/

void PHY_HW_IF_RX_Start(uint8_t PortNum);
void PHY_HW_IF_RX_Stop(uint8_t PortNum);
HAL_StatusTypeDef STUSB16xx_HW_IF_Send_Packet(uint8_t PortNum, uint8_t *pData, uint16_t Size);

void STUSB16xx_HW_IF_Switch_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode);
void HW_IF_STUSB1602_Interrupt_CC_Detection(uint8_t PortNum, FunctionalState status);

/* STUSB16xx_EVAL exported function prototypes */
void PHY_HW_IF_ADCAnalogGPIO_Init(void);
void PHY_HW_IF_ADCAnalogGPIO_DeInit(void);
void PHY_HW_IF_ADCDMA_Init(void);
void PHY_HW_IF_ADCDMA_DeInit(void);
void PHY_HW_IF_TX_Done(uint8_t PortNum);

void STUSB16xx_HW_IF_Alert_Check(uint8_t PortNum);
HAL_StatusTypeDef STUSB16xx_HW_IF_Alert_Manager(uint8_t PortNum);
void STUSB16xx_HW_IF_TX_EN_Status(uint8_t PortNum, GPIO_PinState status);
void STUSB16xx_HW_IF_TX_DMA_Init(uint8_t PortNum);
void STUSB16xx_HW_IF_RX_DMA_Init(uint8_t PortNum);
void STUSB16xx_HW_IF_Set_DMA_Normal_Mode(uint8_t PortNum);
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum);
HAL_StatusTypeDef STUSB16xx_HW_IF_Set_VBus_Monitoring(uint8_t PortNum, uint16_t VBus, uint8_t Hset, uint8_t Lset);

void HW_IF_RESET_CTRL(uint8_t PortNum);
void HW_IF_RESET_GPIO(uint8_t PortNum);

HAL_StatusTypeDef HW_IF_PWR_DigitalGPIO_Init(void);


/* USB PD transaction managed by STUSB16xx */
HAL_StatusTypeDef STUSB16xx_HW_IF_HardReset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode);
HAL_StatusTypeDef STUSB16xx_HW_IF_VConnSwap(uint8_t PortNum);
HAL_StatusTypeDef STUSB16xx_HW_IF_DataRoleSwap(uint8_t PortNum);
void USBPDM1_AssertRp(uint8_t PortNum);
void USBPDM1_AssertRd(uint8_t PortNum);

void USBPDM1_DeAssertRp(uint8_t PortNum);
void USBPDM1_DeAssertRd(uint8_t PortNum);

uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum);
uint16_t HW_IF_PWR_GetCurrent(uint8_t PortNum);
HAL_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage);
FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum);

uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint8_t len);
HAL_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize);
HAL_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum);

void USBPD_HW_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef cc);
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, FunctionalState state, USBPD_PortPowerRole_TypeDef role);
void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);

void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode);

void USBPD_HW_IF_EXTI_Callback(uint16_t GPIO_Pin);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F072_USBPD_HW_IF_H_ */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
