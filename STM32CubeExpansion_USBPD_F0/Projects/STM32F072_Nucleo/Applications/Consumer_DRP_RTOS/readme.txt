/**
  @page USB-C Power Delivery Consumer_DRP_RTOS application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Consumer_DRP_RTOS/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    29-June-2016
  * @brief   Description of the USB-C Power Delivery Consumer_DRP_RTOS application.
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
firmware. It describes how to use USB Power Delivery (USBPD) dual power role
(consumer/provider) application based on the STM32F072RB devices.

In this application the STM32 behaves as a consumer (default mode), a power 
profile negotiation occurs on initial attachment of a port pairs:
 - When attached, CC LEDs (D202, D205) will blink once if connected on CC1, 
   twice if connected on CC2.
 - Blue LED (D203) will be blinking twice each time, to show that the device 
   behaves as a consumer.

Port Pairs are required to negotiate an Explicit Contract, if there is an Explicit 
Contract in place, user can start a power role swap process by pressing USER button 
B1 in the Nucleo board:
 - When the explicit contract is established, VBUS LED (D 204) is on to indicate 
   that the Power Contract was established.
   
The power role swap message shall only be sent and received after an Explicit 
Contract has been established between consumer and provider:
 - When the swap process is completed, Blue LED (D203) will be blinking twice each 
   time, to show that the device behaves now as a consumer.


If the Consumer DRP is supplied by mean of NUCLEO-F072RB voltage regulator, the 
system setting is the following one:
 – On NUCLEO-F072RB board, verify that the jumper JP1 is open, JP5 (PWR) closed on 
   U5V (fitting the pins 1-2), and JP6 (IDD) closed.
 – On P-NUCLEO-USB001 expansion board, open the jumpers JP100, J500, JP501.

If the Consumer DRP is supplied by mean of the VBUS delivered by the Provider 
attached by the USB Type-C cable, the system setting is the following one:
 – On the NUCLEO-F072RB board, the jumper JP1 must be closed, JP5 (PWR) closed on 
   E5V (fitting the pins 2-3), and JP6 (IDD) closed.
 – On P-NUCLEO-USB001 expansion board, while the jumpers J500, JP501 are opened, 
   the jumper JP100 must be set according to the port chosen for supplying the 
   system (fit 2-3 for PORT_0 or 1-2 for PORT_1).
   
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - Consumer_DRP_RTOS/Src/main.c                  Main program
  - Consumer_DRP_RTOS/Src/system_stm32f0xx.c      STM32F0xx system clock configuration file
  - Consumer_DRP_RTOS/Src/stm32f0xx_it.c          Interrupt handlers
  - Consumer_DRP_RTOS/Src/usbpd_dpm.c             DPM layer implementation
  - Consumer_DRP_RTOS/Src/usbpd_pwr_if.c          General power interface configuration
  - Consumer_DRP_RTOS/Inc/main.h                  Main program header file
  - Consumer_DRP_RTOS/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - Consumer_DRP_RTOS/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - Consumer_DRP_RTOS/Inc/usbpd_conf.h            USB-C Power Delivery application Configuration file
  - Consumer_DRP_RTOS/Inc/usbpd_dpm.h             DPM Layer header file
  - Consumer_DRP_RTOS/Inc/FreeRTOSConfig.h        FreeRTOS module configuration file
 

@par Hardware and Software environment

  - This application runs on STM32F072RB devices.
  
  - This example has been tested with STMicroelectronics STM32F072RB-Nucleo RevC
    boards with the P-NUCLEO-USB001 shield RevB connected to CN7 and CN10 connectorsand can 
	be easily tailored to any other supported device and development board.
  
  - Connect the STM32 Nucleo board to the USB-C Power Delivery Provider through
    USB typeC cable to the connector CN0 in the X-NUCLEO shield.
	To test this application, the Provider_DRP_RTOS application can be used as a 
	provider with dual power role capability
	
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
 