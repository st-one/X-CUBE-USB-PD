/**
  ******************************************************************************
  * @file    usbpd_hw_if.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    22-June-2016
  * @brief   This file contains the headers of usbpd_hw_if.h for USB-PD Hardwer 
             Interface layer. This file is specific for each device.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
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
#include "common_defines.h"
#include "usbpd_conf.h"
#include "usbpd_def.h"
#include "usbpd_porthandle.h"
#include "stm32f0xx_it.h"

/* Exported typedef ----------------------------------------------------------*/
 typedef enum
 {
   GPIO0 =      0,	   /**<GPIO0 */
   GPIO1 =      1,	   /**<GPIO1 */
   HRP_P0 =	    GPIO1, /**<Setting high this pin expose Rp resistance on Port 0*/
   GPIO2 =      2,	   /**<GPIO2 */
   ENCC1_P0 =   GPIO2, /**<Setting high this pin Enables CC1 for PD communication on Port 0*/
   GPIO3 =      3,	   /**<GPIO3 */
   HRD_P1 =     GPIO3, /**<Setting high this pin expose Rd resistance on Port 1*/
   GPIO4 =      4,	   /**<GPIO4 */
   ENCC2_P0 =   GPIO4, /**<Setting high this pin Enables CC2 for PD communication on Port 0*/
   GPIO5 =      5,	   /**<GPIO5 */
   GPIO6 =      6,	   /**<GPIO6 */
   HRP_P1 =     GPIO6, /**<Setting high this pin expose Rp resistance on Port 1*/
   GPIO7 =      7,	   /**<GPIO7 */
   HRD_P0 =     GPIO7, /**<Setting high this pin expose Rd resistance on Port 0*/
   GPIO8 =      8,	   /**<GPIO8 */
   ENCC1_P1 =   GPIO8, /**<Setting high this pin Enables CC1 for PD communication on Port 1*/
   GPIO9 =      9,	   /**<GPIO9 */
   ENCC2_P1 =   GPIO9, /**<Setting high this pin Enables CC2 for PD communication on Port 1*/
   GPIO10 =     10,	   /**<GPIO10 */
   GPIO11 =     11,	   /**<GPIO11 */
   PWREN_P0 =   GPIO10,/**<Setting high this pin Enables Load Switch on Port 0*/
   PWRDIS_P0 =  GPIO11,/**<Setting high this pin Enables Discharge on Port 0*/
   GPIO12 =     12,	   /**<GPIO12 */
   PWREN_P1 =   GPIO0, /**<Setting high this pin Enables Load Switch on Port 1*/
   PWRDIS_P1 =  GPIO12,/**<Setting high this pin Enables Discharge on Port 1*/
#ifdef P_NUCLEO_USB001_GPIO13
   GPIO13 = 13,		  /**<GPIO13 */
#endif /*P_NUCLEO_USB001_GPIO13*/
   GPIO14 = 14,		  /**<GPIO14 */
#ifdef P_NUCLEO_USB001_GPIO15
   GPIO15 = 15,		  /**<GPIO15 */
#endif /*P_NUCLEO_USB001_GPIO15*/
#ifndef P_NUCLEO_USB001_USE_USB2
   GPIO16 = 16,		  /**<GPIO16 */
   GPIO17 = 17,	      /**<GPIO17 */
 #endif
 } USBPDM1_GPIO_TypeDef;

/* ADC variables and parameters */
#if (USBPD_PORT_COUNT==1)
#define ADCCONVERTEDVALUES_BUFFER_SIZE          6  /* Size of array containing ADC converted values: 
                                                      set to ADC sequencer number of ranks converted, 
													  to have a rank in each address */
#elif (USBPD_PORT_COUNT==2)
#define ADCCONVERTEDVALUES_BUFFER_SIZE          10 /* Size of array containing ADC converted values: 
                                                      set to ADC sequencer number of ranks converted, 
													  to have a rank in each address */
#endif

/* Exported define -----------------------------------------------------------*/
#ifdef P_NUCLEO_USB001_USE_USB2
 #if defined(P_NUCLEO_USB001_GPIO13) && defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn	        16
 #elif defined(P_NUCLEO_USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn	        15
 #else
  #define USBPDM1_GPIOn	        14
 #endif /*defined*/
#else /*P_NUCLEO_USB001_USE_USB2 not defined*/
 #if defined(P_NUCLEO_USB001_GPIO13) && defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn	        18
 #elif defined(P_NUCLEO_USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn	        17
 #else
  #define USBPDM1_GPIOn         16
 #endif /*defined*/
#endif

/* Power selection pins configuration will be moved to other location in next releases */
#define USBPDM1_POWSELn		    4
 
#define P_NUCLEO_USB001_ADC    ADC1
 
#if (USBPD_PORT_COUNT == 1)
#define USBPDM1_ADCn		   3
#elif (USBPD_PORT_COUNT == 2)
#define USBPDM1_ADCn	       5
#endif

#if (USBPD_PORT_COUNT == 1)
#define USBPDM1_RXDn           2
#elif (USBPD_PORT_COUNT == 2)
#define USBPDM1_RXDn           4
#endif
#define ADCx_CLK_ENABLE()      __HAL_RCC_ADC1_CLK_ENABLE()

#define ADCx_FORCE_RESET()     __HAL_RCC_ADC1_FORCE_RESET()
#define ADCx_RELEASE_RESET()   __HAL_RCC_ADC1_RELEASE_RESET()

/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()  __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA               DMA1_Channel1
#define TX_FREQ                ( 300000 )
#define BMC_TX_FREQ            ( 2 * TX_FREQ )
 
/* Exported constants --------------------------------------------------------*/
#define TX_BUFFER_LEN	        28 /*!< Size of Tx Buffer */

/* Exported macro ------------------------------------------------------------*/
#define DRP_PORT			    GPIOC
#define DRP_PIN				    GPIO_PIN_12
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

#if (USBPD_PORT_COUNT == 1)
#define ENCC1_PIN(__PORT__)		( ENCC1_P0 )
#define ENCC2_PIN(__PORT__)		( ENCC2_P0 )
#elif (USBPD_PORT_COUNT == 2)
#define ENCC1_PIN(__PORT__)		((__PORT__ == 0) ? ENCC1_P0 : ENCC1_P1 )
#define ENCC2_PIN(__PORT__)		((__PORT__ == 0) ? ENCC2_P0 : ENCC2_P1 )
#endif

#define OTHER_CC(__CC__)                ( ((CCxPin_TypeDef)(__CC__))==CC1 ? CC2 : ( (((CCxPin_TypeDef)(__CC__))==CC2) ? CC1 : CCNONE ) ) 

#if (USBPD_PORT_COUNT == 1)
#define VBUS_INDEX(__PORT__)		5
#elif (USBPD_PORT_COUNT == 2)
#define VBUS_INDEX(__PORT__)		((__PORT__ == 0) ? 8 : 9 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define CC1_INDEX(__PORT__)	        0
#elif (USBPD_PORT_COUNT == 2)
#define CC1_INDEX(__PORT__)		    ((__PORT__ == 0) ? 0 : 2 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define CC2_INDEX(__PORT__)	        3
#elif (USBPD_PORT_COUNT == 2)
#define CC2_INDEX(__PORT__)		    ((__PORT__ == 0) ? 5 : 4 )
#endif

/* NEW CAD DEFS */
#ifdef USBPD_SOURCE_ADV_Def_USB
#define threshold_vRa    200           /**< SRC vRa port 1 threshold = 0.2V  Default USB */
#define threshold_vRd    1600          /**< SRC vRd port 1 threshold = 1.6V  Default USB */
#else
#ifdef USBPD_SOURCE_ADV_1_5A_5V
#define threshold_vRa    400           /**< SRC vRa port 1 threshold = 0.4V  1.5 A @ 5 V */
#define threshold_vRd    1600          /**< SRC vRd port 1 threshold = 1.6V  1.5 A @ 5 V */
#else
#define threshold_vRa    800           /**< SRC vRa port 1 threshold = 0.8V  3.0 A @ 5 V */
#define threshold_vRd    2600          /**< SRC vRd port 1 threshold = 2.6V  3.0 A @ 5 V */
#endif
#endif

#define CAD_threshold_SNK_vRd_USB               MV2ADC(200)     /**< SNK vRd threshold = 0.2V  - USB default */
#define CAD_threshold_SNK_vRd_1_5A              MV2ADC(660)     /**< SNK vRd threshold = 0.66V  - 5V 1.5A */ 
#define CAD_threshold_SNK_vRd_3_0A              MV2ADC(1230)    /**< SNK vRd threshold = 1.23V  - 5V 3.0A */ 

#define CAD_threshold_SRC_vRa                   MV2ADC(threshold_vRa)   /**< SRC vRa threshold */
#define CAD_threshold_SRC_vRd                   MV2ADC(threshold_vRd)   /**< SRC vRd threshold */
   
#define CAD_tPDDebounce_threshold               20              /**< tPDDebounce threshold = 20ms  */   
#define CAD_tCCDebounce_threshold               100             /**< tCCDebounce threshold = 100ms  */
   
#define CAD_threshold_VBus                      MV2ADC(500)     /**< Vbus Threshold **/
/* END OF NEW CAD DEFS */

#define BIST_MAX_LENGTH                        (BIST_CARRIER_MODE_MS*600)/(TX_BUFFER_LEN*32)

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Check VBus 
 * @param  port:	port
 */
uint8_t CAD_Check_VBus(uint8_t port);
void USBPD_DMA_PORT1_IRQHandler(void);
void USBPD_DMA_PORT0_IRQHandler(void);
void USBPD_RX_PORT1_Interrupt_IRQHandler(void);
void USBPD_RX_PORT0_Interrupt_IRQHandler(void);
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void);
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void);
void USBPD_PE_PRL_TIMER_IRQHandler(void);

/**
 * @brief  Check CCx HW condition 
 * @param  port: port index
 * @param  CAD_HW_Condition: CCx HW condition
 * @param  CAD_SNK_Source_Current_Adv:  Sink CC pins Multiple Source Current Advertisements 
 * @param  PortPowerRole: Port Power Role: Sink or Device
 */
CCxPin_TypeDef CAD_Check_HW(uint8_t port, CAD_HW_Condition_TypeDef *CAD_HW_Condition, CAD_SNK_Source_Current_Adv_Typedef *CAD_SNK_Source_Current_Adv, USBPD_PortPowerRole_TypeDef PortPowerRole);

/**
  * @brief  Configures P-NUCLEO-USB001 Digital GPIOs.
  * @retval None
  */
void USBPDM1_DigitalGPIO_Init(void);

/**
  * @brief  Turns the selected GPIO On.
  * @param  gpio: Specifies the GPIO to be set on.
  * @retval None
  */
void USBPDM1_GPIO_On(USBPDM1_GPIO_TypeDef gpio);

/**
  * @brief  Turns the selected GPIO Off.
  * @param  gpio: Specifies the GPIO to be set off.
  * @retval None
  */
void USBPDM1_GPIO_Off(USBPDM1_GPIO_TypeDef gpio);

/**
  * @brief  Toggles the selected GPIO.
  * @param  gpio: Specifies the GPIO to be toggled.
  * @retval None
  */
void USBPDM1_GPIO_Toggle(USBPDM1_GPIO_TypeDef gpio);

/**
  * @brief  Enables the selected CCx pin switching the mux.
  * @param  hport: The port handle.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPDM1_Enable_CC(uint8_t hport, CCxPin_TypeDef cc);

/**
  * @brief  Enables the VConn on the port.
  * @param  hport: The port handle.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPDM1_Enable_VConn(uint8_t hport, CCxPin_TypeDef cc);

/**
  * @brief  Set the CCx pin.
  * @param  hport: The port handle.
  * @param  cc: Specifies the ccx to be selected.
  * @retval None
  */
void USBPDM1_Set_CC(uint8_t hport, CCxPin_TypeDef cc);

/**
  * @brief  Set the CCx pin on the TX SPI.
  * @param  hport: The port handle.
  * @param  cc: Specifies the ccx to be selected.
  * @retval None
  */
void USBPDM1_SPI_Set_TX_CC(uint8_t hport, CCxPin_TypeDef cc);

/**
 * @brief  Calculate CRC for the phy payload.
 * @param  Header:	Data buffer to be transmitted
 * @param  pDataObjects: Data buffer to be transmitted
 * @param  DataObjectCount: The number of data object to be transmitted
 * @retval CRC Value
 */
uint32_t USBPDM1_PHY_CRC_Calculate(uint16_t Header, uint32_t * pDataObjects, uint8_t DataObjectCount);

/**
  * @brief  Initialize the ADC Analog GPIOs.
  * @retval None
  */
void USBPDM1_ADCAnalogGPIO_Init(void);

/**
  * @brief  Deinitialize the ADC Analog GPIOs.
  * @retval None
  */
void USBPDM1_ADCAnalogGPIO_DeInit(void);

/**
  * @brief  Initialize the ADC DMA
  * @retval None
  */
void USBPDM1_ADCDMA_Init(void);

/**
  * @brief  Deinitialize the ADC DMA
  * @retval None
  */
void USBPDM1_ADCDMA_DeInit(void);

/**
  * @brief  Initialize the COMP Analog GPIOs.
  * @retval None
  */
void USBPDM1_COMPAnalogGPIO_Init(void);

/**
  * @brief  Deinitialize the COMP Analog GPIOs.
  * @retval None
  */
void USBPDM1_COMPAnalogGPIO_DeInit(void);

/**
  * @brief  Initialize the DMA for the transmission.
  * @param  hport: The port handle.
  * @retval None
  */
void USBPDM1_TX_DMA_Init(uint8_t hport);

/**
  * @brief  Initialize the DMA for the reception.
  * @retval None
  */
void USBPDM1_RX_DMA_Init(uint8_t hport);

/**
  * @brief  Deinitialize the DMA for the reception.
  * @retval None
  */
void USBPDM1_RX_DMA_Deinit(uint8_t hport);

/**
  * @brief  Enable the interrupt for the reception.
  * @param  hport: The handle of the port.
  * @retval None
  */
void USBPDM1_RX_EnableInterrupt(uint8_t hport);

/**
  * @brief  Disable the interrupt for the reception.
  * @param  hport: The handle of the port.
  * @retval None
  */
void USBPDM1_RX_DisableInterrupt(uint8_t hport);

/**
  * @brief  Initialize the variables struct used during the reception decoding.
  * @param  hport: The handle of the port.
  * @retval None
  */
void RX_Init_Hvar(uint8_t hport);

/**
  * @brief  Execute the transmission on the assigned port.
  * @param  hport: The port handle.
  * @retval None
  */
void USBPDM1_TX_Done(uint8_t hport);

/**
  * @brief  Initialize the HW_IF of a specified port.
  * @param  hport: The port handle.
  * @param  cbs: The hw IF callbacks.
  * @param  role: The role of the port.
  * @retval HAL status
  */
HAL_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t hport, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role);

/**
  * @brief  Initialize specific peripheral for the APP.
  * @retval None
  */
void USBPD_HW_IF_GlobalHwInit(void);

/**
  * @brief  Send a Buffer .
  * @usage  The data will be converted in bmc and send through the line
  * @param  hport: The port handle.
  * @param  pBuffer: Data buffer to be transmitted
  * @param  Bitsize: The number of bits to be transmitted
  * @retval HAL status
  */
HAL_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t hport, uint8_t *pBuffer, uint32_t Bitsize);

/**
  * @brief  Enable the VBUS on a specified port.
  * @param  hport: The port handle.
  * @param  state: ENABLE or DISABLE.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t hport, FunctionalState state);

/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  hport: The port handle.
  * @retval FunctionalState
  */
FunctionalState HW_IF_PWR_IsEnabled(uint8_t hport);

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  hport: The port handle.
  * @param  voltage: voltage value to be set.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t hport, uint16_t voltage);

/**
  * @brief  Get the voltage level on a specified port.
  * @param  hport: The port handle.
  * @retval The voltage value 
  */
uint16_t HW_IF_PWR_GetVoltage(uint8_t hport);

/**
  * @brief  Get the current level on a specified port.
  * @param  hport: The port handle.
  * @retval The current value
  */
uint16_t HW_IF_PWR_GetCurrent(uint8_t hport);

/**
  * @brief  Connect the Rp resitors on the CC lines
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_AssertRp(uint8_t hport);

/**
  * @brief  Disconnect the Rp resitors on the CC lines
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_DeAssertRp(uint8_t hport);

/**
  * @brief  Connect the Rd resitors on the CC lines
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_AssertRd(uint8_t hport);

/**
  * @brief  Disconnect the Rd resitors on the CC lines
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_DeAssertRd(uint8_t hport);

/**
  * @brief  Configures the DMA Normal mode
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_Set_DMA_Normal_Mode(uint8_t hport);

/**
  * @brief  Configures the DMA circular mode
  * @param  hport: The port handle.
  * @retval none 
  */
void USBPDM1_Set_DMA_Circular_Mode(uint8_t hport);

/**
  * @brief  Sends the BIST pattern
  * @param  hport: The port handle.
  * @retval none 
  */
HAL_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t hport);

/**
  * @brief  Sends a detachemnt signal.
  * @param  hport: The port handle.
  * @retval none 
  */
void HW_SignalDetachment(uint8_t hport, CCxPin_TypeDef cc);

/**
  * @brief  Stops the Input Channel Timer.
  * @param  htim: Timer handler.
  * @param  Channel: Timer channel.
  * @param  tim_it: timer interrupt.
  * @retval none 
  */
void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F072_USBPD_HW_IF_H_ */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
