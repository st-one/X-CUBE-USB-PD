/**
  ******************************************************************************
  * @file    usbpd_hw_if.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   This file contains the headers of usbpd_hw_if.h for USB-PD Hardwer 
             Interface layer. This file is specific for each device.
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

/* Exported typedef ----------------------------------------------------------*/
 /**
  * @brief Mode how to access functions containing a list of actions  
  */
typedef enum 
{
  ACKNOWLEDGE = 0,
  REQUEST = 1
} USBPD_HardResetMode_TypeDef;

 typedef enum
 {
   GPIO0 =      0,  /**<GPIO0 */
   GPIO1 =      1,  /**<GPIO1 */
   HRP_P0 =  GPIO1,  /**<Setting high this pin expose Rp resistance on Port 0*/
   GPIO2 =      2,  /**<GPIO2 */
   ENCC1_P0 =   GPIO2,  /**<Setting high this pin Enables CC1 for PD communication on Port 0*/
   GPIO3 =      3,  /**<GPIO3 */
   HRD_P1 =     GPIO3,  /**<Setting high this pin expose Rd resistance on Port 1*/
   GPIO4 =      4,  /**<GPIO4 */
   ENCC2_P0 =   GPIO4,  /**<Setting high this pin Enables CC2 for PD communication on Port 0*/
   GPIO5 =      5,  /**<GPIO5 */
   GPIO6 =      6,  /**<GPIO6 */
   HRP_P1 =     GPIO6,  /**<Setting high this pin expose Rp resistance on Port 1*/
   GPIO7 =      7,  /**<GPIO7 */
   HRD_P0 =     GPIO7,  /**<Setting high this pin expose Rd resistance on Port 0*/
   GPIO8 =      8,  /**<GPIO8 */
   ENCC1_P1 =   GPIO8,  /**<Setting high this pin Enables CC1 for PD communication on Port 1*/
   GPIO9 =      9,  /**<GPIO9 */
   ENCC2_P1 =   GPIO9,  /**<Setting high this pin Enables CC2 for PD communication on Port 1*/
   GPIO10 =     10,  /**<GPIO10 */
   GPIO11 =     11,  /**<GPIO11 */
   PWREN_P0 =   GPIO10, /**<Setting high this pin Enables Load Switch on Port 0*/
   PWRDIS_P0 =  GPIO11, /**<Setting high this pin Enables Discharge on Port 0*/
   GPIO12 =     12,  /**<GPIO12 */
   PWREN_P1 =   GPIO0,  /**<Setting high this pin Enables Load Switch on Port 1*/
   PWRDIS_P1 =  GPIO12, /**<Setting high this pin Enables Discharge on Port 1*/
#ifdef P_NUCLEO_USB001_GPIO13
   GPIO13 = 13,    /**<GPIO13 */
#endif /*P_NUCLEO_USB001_GPIO13*/
   GPIO14 = 14,    /**<GPIO14 */
#ifdef P_NUCLEO_USB001_GPIO15
   GPIO15 = 15,    /**<GPIO15 */
#endif /*P_NUCLEO_USB001_GPIO15*/
#ifndef P_NUCLEO_USB001_USE_USB2
   GPIO16 = 16,    /**<GPIO16 */
   GPIO17 = 17,    /**<GPIO17 */
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
  #define USBPDM1_GPIOn  16
#elif defined(P_NUCLEO_USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn  15
#else
  #define USBPDM1_GPIOn  14
#endif /*defined*/
#else /*P_NUCLEO_USB001_USE_USB2 not defined*/
#if defined(P_NUCLEO_USB001_GPIO13) && defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn  18
#elif defined(P_NUCLEO_USB001_GPIO13) || defined(P_NUCLEO_USB001_GPIO15)
  #define USBPDM1_GPIOn  17
#else
  #define USBPDM1_GPIOn  16
#endif /*defined*/
#endif

/* Power selection pins configuration will be moved to other location in next releases */
#define USBPDM1_POWSELn    4
 
#define P_NUCLEO_USB001_ADC        ADC1
 
#if (USBPD_PORT_COUNT == 1)
#define USBPDM1_ADCn    3
#elif (USBPD_PORT_COUNT == 2)
#define USBPDM1_ADCn    5
#endif

#if (USBPD_PORT_COUNT == 1)
#define USBPDM1_RXDn  2
#elif (USBPD_PORT_COUNT == 2)
#define USBPDM1_RXDn  4
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
#define   TX_BUFFER_LEN          28  /*!< Size of Tx Buffer */

/* Exported macro ------------------------------------------------------------*/
#define DRP_PORT      GPIOC
#define DRP_PIN        GPIO_PIN_12
#define DRP_SET_PROVIDER(X)    HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_SET)
#define DRP_SET_CONSUMER(X)    HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_RESET)

/**
 * 	             R603+R604	            ADC->DR    379.9k
 * Vbus(mV) = Vadc * --------- * 3300(mV) * -------
 * 			R604	             4096      49.9k
 * 
 */
#define MVOLT(X)	( ( ( ( ((uint32_t)X)*3799*33 )/499 )*100 ) >>12 )

/**
 * 50Vs - VDDA/2 = Vadcin
 * */
#define MAMP(X)		( ((int32_t)( ( ((uint32_t)X)*3300 )>>10 ) )-6600 )

#if (USBPD_PORT_COUNT == 1)
#define ENCC1_PIN(__PORT__)    ( ENCC1_P0 )
#define ENCC2_PIN(__PORT__)    ( ENCC2_P0 )
#elif (USBPD_PORT_COUNT == 2)
#define ENCC1_PIN(__PORT__)    ((__PORT__ == 0) ? ENCC1_P0 : ENCC1_P1 )
#define ENCC2_PIN(__PORT__)    ((__PORT__ == 0) ? ENCC2_P0 : ENCC2_P1 )
#endif

#define OTHER_CC(__CC__)                ( ((CCxPin_TypeDef)(__CC__))==CC1 ? CC2 : ( (((CCxPin_TypeDef)(__CC__))==CC2) ? CC1 : CCNONE ) ) 

#if (USBPD_PORT_COUNT == 1)
#define VBUS_INDEX(__PORT__)    5
#elif (USBPD_PORT_COUNT == 2)
#define VBUS_INDEX(__PORT__)    ((__PORT__ == 0) ? 8 : 9 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define IBUS_INDEX(__PORT__)		2
#elif (USBPD_PORT_COUNT == 2)
#define IBUS_INDEX(__PORT__)		((__PORT__ == 0) ? 3 : 6 )
#endif
 /*
 Single Port
 P0 CC1 = ADC DMA Index 0
 P0 CC2 = ADC DMA Index  3

  ADC DMA Index => ch
  0     ADC_CHANNEL_0	//P0.CC1    PA0
  1     ADC_CHANNEL_1   //RXRef     PA1
  2     ADC_CHANNEL_3   //P0.IBUS   PA3
  3     ADC_CHANNEL_5   //P0.CC2    PA5
  4     ADC_CHANNEL_10  //Not used  PC0
  5     ADC_CHANNEL_14  //P0.VBUS   PC4 
 
 Dual Port
 P0 CC1 = ADC DMA Index 0
 P0 CC2 = ADC DMA Index 5
 P1 CC1 = ADC DMA Index 2
 P1 CC2 = ADC DMA Index 4
 
  ADC DMA Index => ch
  0     ADC_CHANNEL_0   //P0.CC1    PA0
  1     ADC_CHANNEL_1   //RXRef     PA1
  2     ADC_CHANNEL_2   //P1.CC1    PA2
  3     ADC_CHANNEL_3   //P0.IBUS   PA3
  4     ADC_CHANNEL_4   //P1.CC2    PA4
  5     ADC_CHANNEL_5   //P0.CC2    PA5
  6     ADC_CHANNEL_7   //P1.IBUS   PA7
  7     ADC_CHANNEL_10  //Not used  PC0
  8     ADC_CHANNEL_14  //P0.VBUS   PC4
  9     ADC_CHANNEL_15  //P1.VBUS   PC5
 */
#if (USBPD_PORT_COUNT == 1)
#define CC_ADC_CHANNEL_INDEX(__PORT__, __CC__) ((__CC__) - 1)
#elif (USBPD_PORT_COUNT == 2)
#define CC_ADC_CHANNEL_INDEX(__PORT__, __CC__) (((__PORT__)<<1) + ((__CC__) - 1))
#endif

#define CC_ADC_CHANNEL(__PORT__, __CC__) (USBPDM1_RXD[CC_ADC_CHANNEL_INDEX(__PORT__, __CC__)].ADCCH) 

#define CC_INDEX(__PORT__,__CC__) ((__CC__) == CC1 ? CC1_INDEX(__PORT__) : CC2_INDEX(__PORT__)) 

#if (USBPD_PORT_COUNT == 1)
#define CC1_INDEX(__PORT__)    0
#elif (USBPD_PORT_COUNT == 2)
#define CC1_INDEX(__PORT__)    ((__PORT__ == 0) ? 0 : 2 )
#endif

#if (USBPD_PORT_COUNT == 1)
#define CC2_INDEX(__PORT__)    3
#elif (USBPD_PORT_COUNT == 2)
#define CC2_INDEX(__PORT__)    ((__PORT__ == 0) ? 5 : 4 )
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

#define BUSCHECK_IC_NUMBER                      3               /**< Number of changes to consider the bus busy */      
#define BUSCHECK_THRESH_LOW                     MV2ADC(350)     /**< Low level threshold = 0.35V in check bus idle */

#define CAD_threshold_SNK_vRd_USB               MV2ADC(200)     /**< SNK vRd threshold = 0.2V  - USB default */
#define CAD_threshold_SNK_vRd_1_5A              MV2ADC(660)     /**< SNK vRd threshold = 0.66V  - 5V 1.5A */ 
#define CAD_threshold_SNK_vRd_3_0A              MV2ADC(1230)    /**< SNK vRd threshold = 1.23V  - 5V 3.0A */ 

#define CAD_threshold_SRC_vRa                   MV2ADC(threshold_vRa)   /**< SRC vRa threshold */
#define CAD_threshold_SRC_vRd                   MV2ADC(threshold_vRd)   /**< SRC vRd threshold */
   
#define CAD_tPDDebounce_threshold               20              /**< tPDDebounce threshold = 20ms  */   
#define CAD_tCCDebounce_threshold               100             /**< tCCDebounce threshold = 100ms  */
   
#define CAD_threshold_VBus                      MV2ADC(650)     /**< Vbus Threshold **/
/* END OF NEW CAD DEFS */

#define BIST_MAX_LENGTH                         (BIST_CARRIER_MODE_MS*600)/(TX_BUFFER_LEN*32)

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Check VBus 
 * @param  port:  port
 */
uint8_t CAD_Check_VBus(uint8_t PortNum);
void USBPD_DMA_PORT1_IRQHandler(void);
void USBPD_DMA_PORT0_IRQHandler(void);
void USBPD_RX_PORT1_Interrupt_IRQHandler(void);
void USBPD_RX_PORT0_Interrupt_IRQHandler(void);
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void);
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void);
void ADC1_COMP_IRQHandler(void);

/**
 * @brief  Check CCx HW condition 
 * @param  port: port index
 * @param  CAD_HW_Condition:          CCx HW condition
 * @param  CAD_SNK_Source_Current_Adv:  Sink CC pins Multiple Source Current Advertisements 
 * @param  PortPowerRole: Port Power Role: Sink or Device
 */
CCxPin_TypeDef CAD_Check_HW(uint8_t PortNum, CAD_HW_Condition_TypeDef *CAD_HW_Condition, CAD_SNK_Source_Current_Adv_Typedef *CAD_SNK_Source_Current_Adv, USBPD_PortPowerRole_TypeDef PortPowerRole);

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
  * @param  PortNum The port handle.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPDM1_Enable_CC(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Enables the VConn on the port.
  * @param  PortNum The port handle.
  * @param  cc: Specifies the CCx to be selected (1 or 2).
  * @retval None
  */
void USBPD_HW_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Set the CCx pin.
  * @param  PortNum The port handle.
  * @param  cc: Specifies the ccx to be selected.
  * @retval None
  */
void USBPDM1_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Set the CCx pin on the TX SPI.
  * @param  PortNum The port handle.
  * @param  cc: Specifies the ccx to be selected.
  * @retval None
  */
void USBPDM1_SPI_Set_TX_CC(uint8_t PortNum, CCxPin_TypeDef cc);

/**
 * @brief  Calculate CRC for the phy payload.
 * @param  Header:  Data buffer to be transmitted
 * @param  pDataObjects: Data buffer to be transmitted
 * @param  DataObjectCount:        The number of data object to be transmitted
 * @retval CRC Value
 */
uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint8_t len);

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
  * @param  PortNum The port handle.
  * @retval None
  */
void USBPDM1_TX_DMA_Init(uint8_t PortNum);

/**
  * @brief  Initialize the DMA for the reception.
  * @retval None
  */
void USBPDM1_RX_DMA_Init(uint8_t PortNum);

/**
  * @brief  Deinitialize the DMA for the reception.
  * @retval None
  */
void USBPDM1_RX_DMA_Deinit(uint8_t PortNum);

/**
  * @brief  Enable the interrupt for the reception.
  * @param  PortNum The handle of the port.
  * @retval None
  */
void USBPDM1_RX_EnableInterrupt(uint8_t PortNum);

/**
  * @brief  Disable the interrupt for the reception.
  * @param  PortNum The handle of the port.
  * @retval None
  */
void USBPDM1_RX_DisableInterrupt(uint8_t PortNum);

/**
  * @brief  Initialize the variables struct used during the reception decoding.
  * @param  PortNum The handle of the port.
  * @retval None
  */
void RX_Init_Hvar(uint8_t PortNum);

/**
  * @brief  Execute the transmission on the assigned port.
  * @param  PortNum The port handle.
  * @retval None
  */
void USBPDM1_TX_Done(uint8_t PortNum);

/**
  * @brief  Initialize the HW_IF of a specified port.
  * @param  PortNum The port handle.
  * @param  cbs:    The hw IF callbacks.
  * @param  role:    The role of the port.
  * @retval HAL status
  */
HAL_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role);

/**
  * @brief  Initialize specific peripheral for the APP.
  * @retval None
  */
void USBPD_HW_IF_GlobalHwInit(void);

/**
  * @brief  Send a Buffer .
  * @usage  The data will be converted in bmc and send through the line
  * @param  PortNum The port handle.
  * @param  pBuffer:    Data buffer to be transmitted
  * @param  Bitsize:    The number of bits to be transmitted
  * @retval HAL status
  */
USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize);

/**
  * @brief  Enable the VBUS on a specified port.
  * @param  PortNum The port handle.
  * @param  state   ENABLE or DISABLE.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, FunctionalState state, USBPD_PortPowerRole_TypeDef role);

/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  PortNum The port handle.
  * @retval FunctionalState
  */
FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum);

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  PortNum The port handle.
  * @param  voltage: voltage value to be set.
  * @retval HAL status
  */
HAL_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t voltage);

/**
  * @brief  Get the voltage level on a specified port.
  * @param  PortNum The port handle.
  * @retval The voltage value 
  */
uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum);

/**
  * @brief  Get the current level on a specified port.
  * @param  PortNum The port handle.
  * @retval The current value 
  */
int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum);

/**
  * @brief  Connect the Rp resitors on the CC lines
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_AssertRp(uint8_t PortNum);

/**
  * @brief  Disconnect the Rp resitors on the CC lines
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_DeAssertRp(uint8_t PortNum);

/**
  * @brief  Connect the Rd resitors on the CC lines
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_AssertRd(uint8_t PortNum);

/**
  * @brief  Disconnect the Rd resitors on the CC lines
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_DeAssertRd(uint8_t PortNum);

/**
  * @brief  Configures the DMA Normal mode
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_Set_DMA_Normal_Mode(uint8_t PortNum);

/**
  * @brief  Configures the DMA circular mode
  * @param  PortNum The port handle.
  * @retval none 
  */
void USBPDM1_Set_DMA_Circular_Mode(uint8_t PortNum);

/**
  * @brief  Sends the BIST pattern
  * @param  PortNum The port handle.
  * @retval none 
  */
HAL_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum);

/**
  * @brief  Sends a detachemnt signal.
  * @param  PortNum The port handle.
  * @retval none 
  */
void HW_SignalDetachment(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Stops the Input Channel Timer.
  * @param  htim: Timer handler.
  * @param  Channel: Timer channel.
  * @param  tim_it: timer interrupt.
  * @retval none 
  */
void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);

/**
  * @brief  Enable/Disable the check bus idle.
  * @param  PortNum Current port number.
  * @param  State: State of the check.
  * @retval none 
  */
void USBPD_HW_IF_CheckBusIdleState(uint8_t PortNum, FunctionalState State);

/**
  * @brief  Reset the HW_IF
  * @param  PortNum Current port number.
  * @retval none 
  */
void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HardResetMode_TypeDef Mode);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F072_USBPD_HW_IF_H_ */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
