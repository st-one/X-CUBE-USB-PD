/**
  @page USB-C Power Delivery Provider application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Provider/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    29-June-2016
  * @brief   Description of the USB-C Power Delivery Provider application.
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
based on the STM32F072RB devices.

When the application starts, connecting an USB-PD consumer device (Sink mode) should 
trigger the power negotiation:
 - At start, Role LED (LED D203) is on and VBUS Green LED (D 204) is off.
 - User should plug the USB-C cable on the dedicated connector.
 - When attached, CC LED (D205) is on.
 - The STM32 MCU behaves as a Provider (Source mode), it exchanges Power profiles with 
   the connected device and waits for Power Request message from the attached consumer
 - If the requested power can be met, the STM32 MCU shall send the Accept message followed 
   by PS_RDY message.
 - Once the Explicit Contract established, VBUS LED (D 204) is on to indicate that the 
   Power Contract was established.



The Provider role can be managed with two different supply options that correspond 
to two configuration settings:
- First option: The Provider is supplied by the on-board NUCLEO-F072RB voltage regulator, 
  by mean of a USB Type-A to Mini-B cable plugged to the CN1 connector and then to a PC.
  On NUCLEO-F072RB board, verify that the jumper JP1 is open, JP5 (PWR) closed on U5V 
  (fitting the pins 1-2), and JP6 (IDD) closed.
- Second option: it will permit to manage the VBUS on the selected port, starting from 
  the NUCLEOF072RB USB PWR voltage (CN1 connector):
  If the Provider is equipped with an external board by power connector CN4:
  – On the NUCLEO-F072RB board, the following jumper settings must be guaranteed: JP1 
    closed, JP5 (PWR) closed on E5V (fitting the pins 2-3), and JP6 (IDD) closed.
  – On the P-NUCLEO-USB001 expansion board, the jumpers J500 and JP501 must be left open.
  This setting configuration will permit to the external power board to supply the entire 
  system and, particularly for the USB PD application, to offer a voltage level for the 
  VBUS of the port.


This application has been tested with USB-PD Chromium Twinkie dongle. The Twinkie
dongle is used as a consumer which request the highest power profile.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct operation.

@par Directory contents

  - Provider/Src/main.c                  Main program
  - Provider/Src/system_stm32f0xx.c      STM32F0xx system clock configuration file
  - Provider/Src/stm32f0xx_it.c          Interrupt handlers
  - Provider/Src/usbpd_dpm.c             DPM layer implementation
  - Provider/Src/usbpd_pwr_if.c          General power interface configuration
  - Provider/Inc/main.h                  Main program header file
  - Provider/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - Provider/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - Provider/Inc/usbpd_conf.h            USB-C Power Delivery application Configuration file
  - Provider/Inc/usbpd_dpm.h             DPM Layer header file
 

@par Hardware and Software environment

  - This application runs on STM32F072RB devices.
  
  - This example has been tested with STMicroelectronics STM32F072RB-Nucleo RevC
    board with P-NUCLEO-USB001 shield RevB connected on CN7 and CN10 connectors 
	and can be easily tailored to any other supported device and development board.

  - Connect the STM32 Nucleo board to the USB-C Power Delivery consumer through
    USB typeC cable to the connector CN0 in the X-NUCLEO shield.
	To test this application, the Consumer_RTOS application can be used as a 
	consumer.

  - STM32F072RB-Nucleo RevC Set-up
    - Note that some PCB rework is needed to use the P-NUCLEO-USB001 shield
			- SB48, SB49, SB62 and SB63 must be closed
			- SB13, SB14, SB15 and SB21 must be removed
			- R34 and R36 must be removed
			- JP5 Jumper must be connected to U5V.
		

@par How to use it ?

In order to make the program work, you must do the following:
  - Open EWARM toolchain 
  - Rebuild all files and load your image into target memory
  - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 