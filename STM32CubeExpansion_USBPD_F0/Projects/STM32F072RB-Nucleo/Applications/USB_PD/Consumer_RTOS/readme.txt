/**
  @page USB-C Power Delivery Consumer_RTOS application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Consumer_RTOS/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Description of the USB-C Power Delivery Consumer_RTOS application.
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
firmware. It describes how to use USB Power Delivery (USB-PD) consumer application
based on the STM32F072 devices.

This application provides an example for managing the Port 0 as a Consumer Only port.
When the application starts, connecting an USB-C Power Delivery provider device
(source mode) should trigger the power negotiation:
 - At start, Role LED (LED D203) will be blinking indicating Port role 
  (blink twice indicating Consumer Role).
 - User should plug the USB-C cable on the dedicated connector.
 - When attached, CC LEDs (D205) will blink once if connected on CC1, twice 
   if connected on CC2.
 - Blue LED (D203) will be blinking twice each time, to show that the device behaves 
   as a consumer.
 - The STM32 MCU behaves as a Consumer (Sink mode), it waits for Power Capabilities message 
   from the attached provider. When a Source Capabilities message is received, the STM32 
   starts the evaluation of the received capabilities and check if one of the received power 
   objects can meet its power requirement.
 - While Communicating, VBUS LED (D204) will be blinking.
 - The STM32 shall send the Request message to request the new power level from the offered 
   Source Capabilities.
 - Once the Explicit Contract is established (PS_Ready message received), VBUS LED (D204) 
   is on to indicate that the Power Contract was established.

The system can manage two supply options for the Consumer configuration. 
The first one is supplied by NUCLEO-F072RB, while the second implements a specific 
feature of the USB PD solutions (i.e. when a Consumer is supplied by the Provider by 
mean of its VBUS). Both configurations correspond to two different settings:
If the Consumer is supplied by mean of NUCLEO-F072RB voltage regulator, the system 
setting is the following one:
 - On NUCLEO-F072RB board, verify that the jumper JP1 is open, JP5 (PWR) closed on 
   U5V (fitting the pins 1-2), and JP6 (IDD) closed.
 - On P-NUCLEO-USB001 expansion board, open the jumpers JP100, J500, JP501.
If the Consumer is supplied by mean of the VBUS delivered by the Provider attached 
by the USB Type-C cable, the system setting is the following one:
 - On the NUCLEO-F072RB board, the jumper JP1 must be closed, JP5 (PWR) closed on 
   E5V (fitting the pins 2-3), and JP6 (IDD) closed.
 - On P-NUCLEO-USB001 expansion board, while the jumpers J500, JP501 are opened, 
   the jumper JP100 must be set according to the port chosen for supplying the system 
   (fit 2-3 for PORT_0 or 1-2 for PORT_1).

This application has been tested with USB-PD Test Conformance Tools (Ellisys, MQP, ...).
Vendor Information File, used for Conformance testing and  describing Port capabilities and supported 
options is provided in application directory in STMicroelectronics_Consumer_STM32F072RBT6_VIF.txt file.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct operation.

@par Directory contents

  - Consumer_RTOS/Src/main.c                  Main program
  - Consumer_RTOS/Src/system_stm32f0xx.c      STM32F0xx system clock configuration file
  - Consumer_RTOS/Src/stm32f0xx_hal_msp.c     HAL MSP file
  - Consumer_RTOS/Src/stm32f0xx_it.c          Interrupt handlers
  - Consumer_RTOS/Src/usbpd_dpm.c             DPM layer implementation
  - Consumer_RTOS/Src/usbpd_pwr_if.c          General power interface configuration
  - Consumer_RTOS/Inc/main.h                  Main program header file
  - Consumer_RTOS/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - Consumer_RTOS/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - Consumer_RTOS/Inc/usbpd_conf.h            USB-C Power Delivery application Configuration file
  - Consumer_RTOS/Inc/usbpd_dpm.h             DPM Layer header file
  - Consumer_RTOS/Inc/FreeRTOSConfig.h        FreeRTOS module configuration file
 

@par Hardware and Software environment

  - This application runs on STM32F072 devices.
  
  - This example has been tested with STMicroelectronics STM32F072RB-Nucleo RevC
    board with P-NUCLEO-USB001 shield (RevB or RevC) connected on CN7 and CN10 connectors 
    and can be easily tailored to any other supported device and development board.

  - Use a USB-C Power Delivery cable to connect the STM32 Nucleo board 
   (connector CN0 in the X-NUCLEO shield) to a USB-C Power Delivery Source device.
    To test this application, the Provider_RTOS application can be used as a 
    Provider (Source), on a 2nd board.

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
 