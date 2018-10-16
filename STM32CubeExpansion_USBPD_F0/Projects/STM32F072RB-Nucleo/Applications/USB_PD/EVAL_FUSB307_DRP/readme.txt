/**
  @page USB-C Power Delivery EVAL_FUSB307_DRP application
  
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    EVAL_FUSB307_DRP/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the USB-C Power Delivery EVAL_FUSB307_DRP application.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. All rights reserved.
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
  @endverbatim
  
@par Application Description
Use of the USB Power Delivery (USB-PD) Dual-Role Power (DRP) application running on STM32F072 devices,
in case of TCPM/TCPC architecture. Project configuration is based on USB Power Delivery Specification revision 3.0.

This application provides an example of USB Power Delivery implementation based on TCPM/TCPC architecture.
Application and USBPD Core Stack are located on TCPM side, and are running on STM32F0xx Nucleo device
on OnSemiconductor FUSB307 board. Application is driving TCPC controller through I2C link.

DRP application behavior highlights capability for a Port to handle either provider or consumer 
role, according to connected device. While connected, Power Role swap is also supported :
- When connected to an USB-C provider only device (source mode), the power role swap 
  between the two boards is not possible, DRP board will act automatically as a sink.
- When connected to an USB-C consumer only device (sink mode), the power role swap 
  between the two boards is not possible, DRP board will act automatically as a source.
- When connected to an USB-C with DRP, the power role swap is done each time user button is pressed.

When the application starts, USB-PD has capability of operating as either a Source or Sink.
When connecting to an USB-PD device (source or sink), application should 
be able to detect type of connected device, and adopt corresponding suitable role, in order to
trigger the power negotiation:
 - User should plug the USB-C cable on the dedicated connector.
 - When STM32 MCU side behaves as a Consumer (Sink mode), i.e. when connected to a Source device,
   it waits for Power Capabilities message from the attached provider.
   When a Source Capabilities message is received, the STM32 
   starts the evaluation of the received capabilities and check if one of the received power 
   objects can meet its power requirement.
   The STM32 shall send the Request message to request the new power level from the offered 
   Source Capabilities.
   Once the PS_RDY message is received, Explicit Contract is established.
 - When STM32 MCU side behaves as a Provider (Source mode), i.e. when connected to a Sink device,
   it exchanges Power profiles with the connected device and waits for Power Request message 
   from the attached consumer.
   If the requested power can be met, the STM32 MCU shall send the Accept message followed 
   by PS_RDY message.
   Explicit Contract is then considered as established.

This application has been tested with USB-PD Test Conformance Tools (Ellisys, MQP, ...).
Vendor Information File, used for Conformance testing and  describing Port capabilities and supported 
options is provided in application directory in STMicroelectronics_EVAL_FUSB307_DRP_STM32F072RBT6_VIF.txt file.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct operation.

@par Directory contents

  - EVAL_FUSB307_DRP/Src/main.c                      Main program
  - EVAL_FUSB307_DRP/Src/system_stm32f0xx.c          STM32F0xx system clock configuration file
  - EVAL_FUSB307_DRP/Src/stm32f0xx_hal_msp.c         HAL MSP file
  - EVAL_FUSB307_DRP/Src/stm32f0xx_it.c              Interrupt handlers
  - EVAL_FUSB307_DRP/Src/usbpd_dpm_user.c            DPM layer implementation
  - EVAL_FUSB307_DRP/Src/usbpd_pwr_if.c              General power interface configuration
  - EVAL_FUSB307_DRP/Inc/main.h                      Main program header file
  - EVAL_FUSB307_DRP/Inc/stm32f0xx_it.h              Interrupt handlers header file
  - EVAL_FUSB307_DRP/Inc/stm32f0xx_hal_conf.h        HAL configuration file
  - EVAL_FUSB307_DRP/Inc/usbpd_dpm_conf.h            USB-C Power Delivery application Configuration file
  - EVAL_FUSB307_DRP/Inc/usbpd_dpm_user.h            DPM Layer header file
  - EVAL_FUSB307_DRP/Inc/usbpd_pdo_defs.h            PDO definition central header file
  - EVAL_FUSB307_DRP/Inc/usbpd_pdo_defs_Drp_1Port.h  1 Port DRP PDO definition file
  - EVAL_FUSB307_DRP/Inc/FreeRTOSConfig.h            FreeRTOS module configuration file
 

@par Hardware and Software environment

  - This application runs on STM32F072 devices, implementing the TCPM task.
  
  - This example has been tested with OnSemiconductor FUSB307 board. Application is to be downloaded 
    on STM32F072 device present on OnSemiconductor evaluation board. Download could be achieved :
    - either by using an external ST-LINK component connected to OnSemiconductor FUSB307 EVB (J5)
    - either by using download procedure as described by OnSemiconductor in "GUI for use with the FUSB307B Evaluation Board" document.

  - Use a USB-C Power Delivery cable to connect the USB-PD Type-C connector on the TCPC controller OnSemi FUSB307 board
    to a USB-C Power Delivery device (Source or Sink).
    To test this application, the Consumer_RTOS application can be used as a 
    Consumer (Sink) or Provider_RTOS application can be used as a Provider (Source), on a 2nd board.

@par How to use it ?

In order to make the program work, you must do the following:
  - Open your preferred toolchain 
  - Rebuild all files and load your image into target memory
  - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 