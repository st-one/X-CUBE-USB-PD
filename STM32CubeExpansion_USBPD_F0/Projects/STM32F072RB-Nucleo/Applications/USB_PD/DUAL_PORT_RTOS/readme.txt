/**
  @page USB-C Power Delivery DUAL_PORT_RTOS application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    DUAL_PORT_RTOS/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Description of the USB-C Power Delivery DUAL_PORT_RTOS application.
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. All rights reserved.
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
This application is a part of the USB Power Delivery package using STM32Cube 
firmware. It describes how to use USB Power Delivery (USB-PD) Dual Port application
based on the STM32F072 devices.

This application provides an example for managing both Port 0 and Port 1 simultaneously.
Both ports are Dual-Role Power (DRP) ports, and behave independently.
User should plug the USB-C cable on the dedicated connector (Port 0: CN0 or Port 1: CN1).
When the application starts, each Port has Capability of operating as either a Source or Sink.
When connecting an USB-C Power Delivery device (source or sink) on a given port, application should 
be able to detect type of connected device, and adopt corresponding suitable role, in order to
trigger the power negotiation:
 - At start, Role LED (LED D203 for Port 0 and LED D200 for Port 1) will be blinking indicating
   Port role (blink three times indicating DRP Role).
 - User should plug the USB-C cable on the dedicated connector (CN0 for Port 0 and CN1 for Port 1).
 - When Port behaves as a Consumer (Sink mode), i.e. when connected to a Source device,
   it waits for Power Capabilities message from the attached provider.
   When a Source Capabilities message is received, the STM32 
   starts the evaluation of the received capabilities and check if one of the received power 
   objects can meet its power requirement.
   The STM32 shall send the Request message to request the new power level from the offered 
   Source Capabilities.
   Once the Explicit Contract established (PS_Ready) message received), 
   VBUS LED (LED D204 for Port 0 and LED D201 for Port 1) is on to indicate
   that the Power Contract was established.
 - When Port behaves as a Provider (Source mode), i.e. when connected to a Source device,
   it exchanges Power profiles with the connected device and waits for Power Request message 
   from the attached consumer.
   If the requested power can be met, the STM32 MCU shall send the Accept message followed 
   by PS_RDY message.
   Once the Explicit Contract established, VBUS LED (LED D204 for Port 0 and LED D201 for Port 1) is on to indicate that the 
   Power Contract was established.

 - When attached, and before Explicit contract is established, VBUS LED 
   (LED D204 for Port 0 and LED D201 for Port 1) will be blinking.
 - When attached, CC LEDs (LED D205 for Port 0 and LED D202 for Port 1) will blink once 
   if connected on CC1, twice if connected on CC2.
 - Role (Blue) LED (LED D203 for Port 0 and LED D200 for Port 1) will be blinking 
   twice each time, if device behaves as a consumer, 
   or will be blinking once each time, if device behaves as a provider.


This application also embeds a Command Line Interface (CLI) feature, which allows user
to get status of Power Delivery application running on Port 0 and Port 1 and to interact
with application through a serial communication.
Please refer to UM2051 (Getting started with the STM32 Nucleo pack for USB Type-C™ and Power Delivery)
for more details on :
 - Boards configuration for allowing CLI use
 - HyperTerminal configuration
 - Available CLI command lines and parameters.
In this application, both ports (Port 0 and Port 1) could be accessed by CLI commands.

The system can manage two supply options for Port 0 and port 1, depending upon Role that Port will
take, after being connected to another device (either Provider or Consumer). 

- First option: System is supplied by the on-board STM32F072RB-Nucleo RevC voltage regulator, 
  by mean of a USB Type-A to Mini-B cable plugged to the CN1 connector and then to a PC.
  On STM32F072RB-Nucleo RevC board, verify that the jumper JP1 is open, JP5 (PWR) closed on U5V 
  (fitting the pins 1-2), and JP6 (IDD) closed.
  On MB1257 expansion board, close the jumper related to the Provider Port (J500 for PORT_0).
  This setting allows to manage the VBUS on the selected port, starting from 
  the STM32F072RB-Nucleo RevC USB PWR voltage (CN1 connector).
- Second option: System is equipped with an external board by power connector CN4:
  On the STM32F072RB-Nucleo RevC board, the following jumper settings must be guaranteed: JP1 
  closed, JP5 (PWR) closed on E5V (fitting the pins 2-3), and JP6 (IDD) closed.
  On P-NUCLEO-USB001 expansion board, while the jumpers J500, JP501 are opened, 
  the jumper JP100 must be set according to the port chosen for supplying the system 
  (fit 2-3 for PORT_0 or 1-2 for PORT_1).
  In Provider case, this setting configuration will permit to the external power board to supply the entire 
  system and, particularly for the USB PD application, to offer a voltage level for the 
  VBUS of the port.
  In Consumer case, Consumer is supplied by mean of the VBUS delivered by the Provider attached 
  by the USB Type-C cable.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct operation.

@par Directory contents

  - DUAL_PORT_RTOS/Src/main.c                  Main program
  - DUAL_PORT_RTOS/Src/system_stm32f0xx.c      STM32F0xx system clock configuration file
  - DUAL_PORT_RTOS/Src/stm32f0xx_hal_msp.c     HAL MSP file
  - DUAL_PORT_RTOS/Src/stm32f0xx_it.c          Interrupt handlers
  - DUAL_PORT_RTOS/Src/usbpd_dpm.c             DPM layer implementation
  - DUAL_PORT_RTOS/Src/usbpd_pwr_if.c          General power interface configuration
  - DUAL_PORT_RTOS/Inc/main.h                  Main program header file
  - DUAL_PORT_RTOS/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - DUAL_PORT_RTOS/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - DUAL_PORT_RTOS/Inc/usbpd_conf.h            USB-C Power Delivery application Configuration file
  - DUAL_PORT_RTOS/Inc/usbpd_dpm.h             DPM Layer header file
  - DUAL_PORT_RTOS/Inc/FreeRTOSConfig.h        FreeRTOS module configuration file
 

@par Hardware and Software environment

  - This application runs on STM32F072 devices.
  
  - This example has been tested with STMicroelectronics STM32F072RB-Nucleo RevC
    board with P-NUCLEO-USB001 shield (RevB or RevC) connected on CN7 and CN10 connectors 
    and can be easily tailored to any other supported device and development board.

  - Connect the STM32 Nucleo board to the USB-C Power Delivery consumer through
    USB typeC cable to either CN0 or CN1 connectors in the X-NUCLEO shield.
    To test this application on Port 0 or Port 1, a 2nd board could be loaded with 
    the Consumer_RTOS application to be used as a Consumer (Sink) or with 
    the Provider_RTOS application can be used as a Provider (Source).

  - STM32F072RB-Nucleo RevC Set-up
    - Note that some PCB rework is needed to use the P-NUCLEO-USB001 shield
      - SB48, SB49, SB62 and SB63 must be closed
      - SB13, SB14, SB15 and SB21 must be removed
      - R34 and R36 must be removed
      - JP5 Jumper must be connected to U5V.

@par How to use it ?

In order to make the program work, you must do the following:
  - Open your preferred toolchain 
  - Rebuild all files and load your image into target memory
  - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 