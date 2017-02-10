/**
  @page USB-C Power Delivery Provider_CLI_RTOS application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Provider_CLI_RTOS/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Description of the USB-C Power Delivery Provider_CLI_RTOS application.
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
firmware. It describes how to use USB Power Delivery (USB-PD) provider application
based on the STM32F072 devices  and embeds support of a Command Line Interface.

This application provides an example for managing the Port 0 as a Provider Only port.
When the application starts, connecting an USB-C Power Delivery consumer device
(Sink mode) should trigger the power negotiation:
 - At start, Role LED (LED D203) will be blinking indicating Port role 
  (blink once indicating Provider Role).
 - User should plug the USB-C cable on the dedicated connector.
 - When attached, CC LEDs (D205) will blink once if connected on CC1, twice 
   if connected on CC2.
 - Blue LED (D203) will be blinking one time, to show that the device behaves as 
   a provider.
 - The STM32 MCU behaves as a Provider (Source mode), it exchanges Power profiles with 
   the connected device and waits for Power Request message from the attached consumer
 - When attached, and before Explicit contract is established, VBUS LED (D204) will be blinking.
 - If the requested power can be met, the STM32 MCU shall send the Accept message followed 
   by PS_RDY message.
 - Once the Explicit Contract established, VBUS LED (D204) is on to indicate that the 
   Power Contract was established.

This application also embeds a Command Line Interface (CLI) feature, which allows user
to get status of the Power Delivery application running on Port 0 and to act on it 
through a serial communication.
Please refer to UM2051 (Getting started with the STM32 Nucleo pack for USB Type-C™ and Power Delivery)
for more details on :
 - Boards configuration for allowing CLI use
 - HyperTerminal configuration
 - Available CLI command lines and parameters.
In this application, only one port (Port 0) is handled and is then accessed by CLI commands.

The Provider role can be managed with two different supply options that correspond 
to two configuration settings:
- First option: The Provider is supplied by the on-board STM32F072RB-Nucleo RevC voltage regulator, 
  by mean of a USB Type-A to Mini-B cable plugged to the CN1 connector and then to a PC.
  On STM32F072RB-Nucleo RevC board, verify that the jumper JP1 is open, JP5 (PWR) closed on U5V 
  (fitting the pins 1-2), and JP6 (IDD) closed.
  On MB1257 expansion board, close the jumper related to the Provider Port,
  (J500 for PORT_0 and JP501 for PORT_1).
  This setting allows to manage the VBUS on the selected port, starting from 
  the STM32F072RB-Nucleo RevC USB PWR voltage (CN1 connector).
- Second option: The Provider is equipped with an external board by power connector CN4:
  On the STM32F072RB-Nucleo RevC board, the following jumper settings must be guaranteed: JP1 
  closed, JP5 (PWR) closed on E5V (fitting the pins 2-3), and JP6 (IDD) closed.
  On the P-NUCLEO-USB001 expansion board, the jumpers JP100, J500 and JP501 must be left open.
  This setting configuration will permit to the external power board to supply the entire 
  system and, particularly for the USB PD application, to offer a voltage level for the 
  VBUS of the port.


This application has been tested with USB-PD Test Conformance Tools (Ellisys, MQP, ...).
Vendor Information File, used for Conformance testing and  describing Port capabilities and supported 
options is provided in application directory in STMicroelectronics_Provider_STM32F072RBT6_VIF.txt file.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct operation.

@par Directory contents

  - Provider_CLI_RTOS/Src/main.c                  Main program
  - Provider_CLI_RTOS/Src/system_stm32f0xx.c      STM32F0xx system clock configuration file
  - Provider_CLI_RTOS/Src/stm32f0xx_hal_msp.c     HAL MSP file
  - Provider_CLI_RTOS/Src/stm32f0xx_it.c          Interrupt handlers
  - Provider_CLI_RTOS/Src/usbpd_dpm.c             DPM layer implementation
  - Provider_CLI_RTOS/Src/usbpd_pwr_if.c          General power interface configuration
  - Provider_CLI_RTOS/Inc/main.h                  Main program header file
  - Provider_CLI_RTOS/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - Provider_CLI_RTOS/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - Provider_CLI_RTOS/Inc/usbpd_conf.h            USB-C Power Delivery application Configuration file
  - Provider_CLI_RTOS/Inc/usbpd_dpm.h             DPM Layer header file
  - Provider_CLI_RTOS/Inc/FreeRTOSConfig.h        FreeRTOS module configuration file
 

@par Hardware and Software environment

  - This application runs on STM32F072 devices.
  
  - This example has been tested with STMicroelectronics STM32F072RB-Nucleo RevC
    board with P-NUCLEO-USB001 shield (RevB or RevC) connected on CN7 and CN10 connectors 
    and can be easily tailored to any other supported device and development board.

  - Use a USB-C Power Delivery cable to connect the STM32 Nucleo board 
   (connector CN0 in the X-NUCLEO shield) to a USB-C Power Delivery Consumer device.
    To test this application, the Consumer_RTOS application can be used as a 
    consumer, on a 2nd board.

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
 